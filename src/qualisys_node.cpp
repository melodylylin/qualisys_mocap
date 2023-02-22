#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "qualisys_cpp_sdk/RTProtocol.h"
#include "qualisys_cpp_sdk/RTPacket.h"
#include "qualisys_cpp_sdk/Network.h"

using namespace std::chrono_literals;


class QualisysNode : public rclcpp::Node
{
  private:
    typedef rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePubPtr;
    std::map<std::string, posePubPtr> pub_pose_{};
    CRTProtocol rtProtocol_;
    std::string serverAddr_{"127.0.0.1"};
    const unsigned short basePort_{22222};
    const int majorVersion_{1};
    const int minorVersion_{19};
    const bool bigEndian_{false};
    unsigned short udpPort_{6734};

  public:
    QualisysNode()
    : Node("qualisys_node") {
      serverAddr_ = "192.168.123.2";
      run();
    }

    ~QualisysNode() {
      rtProtocol_.StopCapture();
      rtProtocol_.Disconnect();
    }

  private:
    void run() {
        bool dataAvailable{false};
        bool streamFrames{false};

        while (rclcpp::ok()) {
          if (!rtProtocol_.Connected())
          {
              if (!rtProtocol_.Connect(serverAddr_.c_str(), basePort_, &udpPort_, majorVersion_, minorVersion_, bigEndian_))
              {
                  printf("rtProtocol_.Connect: %s\n\n", rtProtocol_.GetErrorString());
                  sleep(1);
                  continue;
              }
          }

          if (!dataAvailable)
          {
              if (!rtProtocol_.Read6DOFSettings(dataAvailable))
              {
                  printf("rtProtocol_.Read6DOFSettings: %s\n\n", rtProtocol_.GetErrorString());
                  sleep(1);
                  continue;
              }
          }

          if (!streamFrames)
          {
              if (!rtProtocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udpPort_, NULL, CRTProtocol::cComponent6d))
              {
                  printf("rtProtocol_.StreamFrames: %s\n\n", rtProtocol_.GetErrorString());
                  sleep(1);
                  continue;
              }
              streamFrames = true;

              printf("Starting to streaming 6DOF data\n\n");
          }

          CRTPacket::EPacketType packetType;

          if (rtProtocol_.Receive(packetType, true) == CNetwork::ResponseType::success)
          {
              if (packetType == CRTPacket::PacketData)
              {
                  float fX, fY, fZ;
                  float rotationMatrix[9];

                  CRTPacket* rtPacket = rtProtocol_.GetRTPacket();

                  printf("Frame %d\n", rtPacket->GetFrameNumber());
                  printf("======================================================================================================================\n");

                  for (unsigned int i = 0; i < rtPacket->Get6DOFBodyCount(); i++)
                  {
                      if (rtPacket->Get6DOFBody(i, fX, fY, fZ, rotationMatrix))
                      {
                          const char* pTmpStr = rtProtocol_.Get6DOFBodyName(i);
                          if (pTmpStr)
                          {
                              printf("%-12s ", pTmpStr);
                          }
                          else
                          {
                              printf("Unknown     ");
                          }
                          printf("Pos: %9.3f %9.3f %9.3f    Rot: %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
                              fX, fY, fZ, rotationMatrix[0], rotationMatrix[1], rotationMatrix[2],
                              rotationMatrix[3], rotationMatrix[4], rotationMatrix[5], rotationMatrix[6], rotationMatrix[7], rotationMatrix[8]);

                          if (pTmpStr) {
                            if (pub_pose_.find(pTmpStr) == pub_pose_.end()) {
                              pub_pose_[pTmpStr] = this->create_publisher<geometry_msgs::msg::PoseStamped>(pTmpStr, 10);
                            }
                            // publish body
                            geometry_msgs::msg::PoseStamped msg;
			    msg.header.stamp = rclcpp::Clock().now();
			    msg.header.frame_id = "qualisys";
			    msg.pose.position.x = fX;
			    msg.pose.position.y = fY;
			    msg.pose.position.z = fZ;
			    msg.pose.orientation.x = 0;
			    msg.pose.orientation.y = 0;
			    msg.pose.orientation.z = 0;
			    msg.pose.orientation.w = 1;
                            pub_pose_[pTmpStr]->publish(msg);
                          }
                      }
                  }
                  printf("\n");
              }
          }
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QualisysNode>());
  rclcpp::shutdown();
  return 0;
}

