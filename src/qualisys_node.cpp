#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "qualisys_cpp_sdk/RTProtocol.h"
#include "qualisys_cpp_sdk/RTPacket.h"
#include "qualisys_cpp_sdk/Network.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class QualisysNode : public rclcpp::Node
{
private:
    typedef rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePubPtr;
    typedef rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPubPtr;
    typedef rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPubPtr;
    typedef std::vector<geometry_msgs::msg::Point> Markers;
    std::map<std::string, odomPubPtr> pub_odom_{};
    std::map<std::string, posePubPtr> pub_pose_{};
    std::map<std::string, markerPubPtr> pub_marker_{};
    std::vector<std::string> rb_names_{};
    CRTProtocol rtProtocol_;
    std::string serverAddr_{"127.0.0.1"};
    const unsigned short basePort_{22222};
    const int majorVersion_{1};
    const int minorVersion_{22};
    const bool bigEndian_{false};
    unsigned short udpPort_{6734};
    std::string frameComponents_{"3D 6D"};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
    QualisysNode()
        : Node("qualisys_node")
    {
        declare_parameter("mocap_server_ip", "192.168.123.2");
        get_parameter("mocap_server_ip", serverAddr_);
        RCLCPP_INFO(this->get_logger(),"Qualisys Mocap Server IP: %s", serverAddr_.c_str());
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        run();
    }

    ~QualisysNode()
    {
        rtProtocol_.StopCapture();
        rtProtocol_.Disconnect();
    }

private:
    void run()
    {
        bool dataAvailable{false};
        bool streamFrames{false};

        while (rclcpp::ok())
        {
            if (!rtProtocol_.Connected())
            {
                if (!rtProtocol_.Connect(serverAddr_.c_str(), basePort_, &udpPort_, majorVersion_, minorVersion_, bigEndian_))
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rtProtocol_.Connect: %s\n\n", rtProtocol_.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!dataAvailable)
            {
                if (!rtProtocol_.Read6DOFSettings(dataAvailable))
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rtProtocol_.Read6DOFSettings: %s\n\n", rtProtocol_.GetErrorString());
                    sleep(1);
                    continue;
                }

                if (!rtProtocol_.Read3DSettings(dataAvailable))
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rtProtocol_.Read3DSettings: %s\n\n", rtProtocol_.GetErrorString());
                    sleep(1);
                    continue;
                }
            }

            if (!streamFrames)
            {
                // if (!rtProtocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort_, NULL, CRTProtocol::cComponent6d))
                if (!rtProtocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort_, NULL, frameComponents_.c_str()))
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rtProtocol_.StreamFrames: %s\n\n", rtProtocol_.GetErrorString());
                    sleep(1);
                    continue;
                }
                streamFrames = true;

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting to streaming mocap 3D/6DOF data");
            }

            CRTPacket::EPacketType packetType;

            if (rtProtocol_.Receive(packetType, true) == CNetwork::ResponseType::success)
            {
                if (packetType == CRTPacket::PacketData)
                {
                    float fX, fY, fZ;
                    float rotationMatrix[9];

                    CRTPacket *rtPacket = rtProtocol_.GetRTPacket();

                    // printf("Frame %d\n", rtPacket->GetFrameNumber());
                    // printf("======================================================================================================================\n");
                    // printf("rbs: %d\nmarkers: %d\n", rtPacket->Get6DOFBodyCount(), rtPacket->Get3DMarkerCount());

                    for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                    {
                        if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                        {
                            const char *pTmpStr = rtProtocol_.Get6DOFBodyName(i);
                            if (pTmpStr)
                            {
                                rb_names_.push_back(pTmpStr);
                                // printf("%-12s ", pTmpStr);
                            }
                            else
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown     ");
                            }
                            // printf("Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
                            //     fX, fY, fZ, rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
                            //     rotationMatrix[3], rotationMatrix[4], rotationMatrix[5], rotationMatrix[6], rotationMatrix[7], rotationMatrix[8]);

                            if (pTmpStr && !std::isnan(fX))
                            {
                                if (pub_pose_.find(pTmpStr) == pub_pose_.end())
                                {
                                    pub_pose_[pTmpStr] = this->create_publisher<geometry_msgs::msg::PoseStamped>(std::string{pTmpStr}+"/pose", 10);
                                }

                                if (pub_odom_.find(pTmpStr) == pub_odom_.end())
                                {
                                    pub_odom_[pTmpStr] = this->create_publisher<nav_msgs::msg::Odometry>(std::string{pTmpStr}+"/odom", 10);
                                }


                                // convert to quaternion
                                tf2::Matrix3x3 R(
                                    rotationMatrix[0], rotationMatrix[3], rotationMatrix[6],
                                    rotationMatrix[1], rotationMatrix[4], rotationMatrix[7],
                                    rotationMatrix[2], rotationMatrix[5], rotationMatrix[8]);
                                tf2::Quaternion q;
                                R.getRotation(q);

                                // publish body
                                {
                                    geometry_msgs::msg::PoseStamped msg;
                                    msg.header.stamp = rclcpp::Clock().now();
                                    msg.header.frame_id = "qualisys";
                                    msg.pose.position.x = fX / 1000.0;
                                    msg.pose.position.y = fY / 1000.0;
                                    msg.pose.position.z = fZ / 1000.0;
                                    msg.pose.orientation.x = q.x();
                                    msg.pose.orientation.y = q.y();
                                    msg.pose.orientation.z = q.z();
                                    msg.pose.orientation.w = q.w();
                                    pub_pose_[pTmpStr]->publish(msg);
                                }

                                // publish odometry
                                {
                                    nav_msgs::msg::Odometry msg;
                                    msg.header.stamp = rclcpp::Clock().now();
                                    msg.header.frame_id = "qualisys";
                                    msg.child_frame_id = pTmpStr;
                                    msg.pose.pose.position.x = fX / 1000.0;
                                    msg.pose.pose.position.y = fY / 1000.0;
                                    msg.pose.pose.position.z = fZ / 1000.0;
                                    msg.pose.pose.orientation.x = q.x();
                                    msg.pose.pose.orientation.y = q.y();
                                    msg.pose.pose.orientation.z = q.z();
                                    msg.pose.pose.orientation.w = q.w();
                                    pub_odom_[pTmpStr]->publish(msg);
                                }

                                // TF message
                                {
                                    geometry_msgs::msg::TransformStamped tf_msg;

                                    // Read message content and assign it to
                                    // corresponding tf variables
                                    tf_msg.header.stamp = rclcpp::Clock().now();
                                    tf_msg.header.frame_id = "qualisys";
                                    tf_msg.child_frame_id = pTmpStr;

                                    tf_msg.transform.translation.x = fX / 1000.0;
                                    tf_msg.transform.translation.y = fY / 1000.0;
                                    tf_msg.transform.translation.z = fZ / 1000.0;                       
                                    tf_msg.transform.rotation.x = q.x();
                                    tf_msg.transform.rotation.y = q.y();
                                    tf_msg.transform.rotation.z = q.z();
                                    tf_msg.transform.rotation.w = q.w();

                                    // Send the transformation
                                    tf_broadcaster_->sendTransform(tf_msg);
                                }
                            }
                        }
                    }
                    // printf("\n");
                    
                    std::map<std::string, Markers> rb_marker_map_;
                    for (unsigned int i = 0 ; i < rtPacket->Get3DMarkerCount(); i++)
                    {
                        if (rtPacket->Get3DMarker(i, fX, fY, fZ))
                        {
                            std::string rb_name;
                            // int marker_idx;
                            std::vector<std::string> substr_list;
                            std::string delimiter = " - ";
                            const char *pTmpStr = rtProtocol_.Get3DLabelName(i);
                            if (pTmpStr && !std::isnan(fX))
                            {
                                // printf("%-12s ", pTmpStr);
                                std::string str{pTmpStr};

                                int start = 0;
                                int end = str.find(delimiter);
                                
                                while (end != -1)
                                {
                                    substr_list.push_back(std::string(str.substr(0, end)));
                                    start = end + delimiter.size();
                                    end = str.find(delimiter, start);
                                }
                                substr_list.push_back(std::string(str.substr(start, end)));

                                rb_name = substr_list.front();
                                // marker_idx = (int)substr_list.back();

                                if (rb_marker_map_.find(rb_name) == rb_marker_map_.end())
                                {
                                    rb_marker_map_[rb_name] = Markers{};
                                }
                                
                                // std::cout << "filling markers...\n";
                                geometry_msgs::msg::Point point;
                                point.x = fX / 1000.0;
                                point.y = fY / 1000.0;
                                point.z = fZ / 1000.0;
                                rb_marker_map_[rb_name].push_back(point);
                            }
                            else
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown    ");
                            }
                            // printf("x: %9.3f y: %9.3f z: %9.3f\n", fX, fY, fZ);
                        }
                    }
                    for (auto i = rb_marker_map_.begin(); i != rb_marker_map_.end(); i++)
                    {
                        if (pub_marker_.find(i->first) == pub_marker_.end())
                        {
                            pub_marker_[i->first] = this->create_publisher<visualization_msgs::msg::Marker>(i->first+"/markers", 10);
                            // std::cout << "Marker pub created: " << i->first << "\n";
                        }
                        visualization_msgs::msg::Marker marker_msg;
                        marker_msg.header.stamp = rclcpp::Clock().now();
                        marker_msg.header.frame_id = "qualisys";
                        marker_msg.type = visualization_msgs::msg::Marker::POINTS;
                        marker_msg.points = i->second;
                        marker_msg.scale.x = .03;
                        marker_msg.scale.y = .03;
                        marker_msg.color.g = 1.0;
                        marker_msg.color.a = 1.0;
                        marker_msg.action = visualization_msgs::msg::Marker::ADD;
                        pub_marker_[i->first]->publish(marker_msg);
                    }
                }
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QualisysNode>());
    rclcpp::shutdown();
    return 0;
}
