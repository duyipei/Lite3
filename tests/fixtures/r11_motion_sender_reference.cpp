// Offline fixture: the pinned upstream callback below is executed with a fake
// sendto. There are no sockets, ROS nodes, or robot commands.
// Upstream: DeepRoboticsLab/Lite3_ROS@e4ab5dc1c7f3ff46274162a1ef302253936578aa
/*
MIT License
Copyright (c) 2023 DeepRoboticsLab
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <array>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>
#include <sys/types.h>
#include "r11_foxy_one_d_velocity_iterator.hpp"
#define RCLCPP_INFO(...) ((void)0)
struct sockaddr { char data[16]; };
struct sockaddr_in { char data[16]; };
namespace geometry_msgs { namespace msg {
struct Twist { using SharedPtr=std::shared_ptr<Twist>;
 struct V {double x=0,y=0,z=0;}; V linear,angular; };
}}
#pragma pack(push,4)
struct MotionSimpleCMD {int32_t cmd_code,size,type;};
struct MotionComplexCMD:MotionSimpleCMD {double data;};
#pragma pack(pop)
static_assert(sizeof(MotionComplexCMD)==20,"reference ABI changed");
std::vector<std::array<unsigned char,20>> packets;
ssize_t sendto(int,const void* p,size_t n,int,const sockaddr*,size_t) {
 std::array<unsigned char,20> data{};
 if(n!=20) return -1;
 std::memcpy(data.data(),p,n);packets.push_back(data);return n;
}
class MockSender {
public:
 int socked_fd_=0; sockaddr_in remote_addr_{}; MotionComplexCMD complex_cmd_{};
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        RCLCPP_INFO(get_logger(),"recevied cmd_vel #############");
        complex_cmd_.cmd_code = 320;
        complex_cmd_.size = 8;
        complex_cmd_.type = 1;
        complex_cmd_.data = msg->linear.x;                  ///< linear velocity
        ssize_t bytesSent = sendto(socked_fd_, &complex_cmd_, sizeof(complex_cmd_), 0,
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        RCLCPP_INFO(get_logger(),"sender linear.x sucessful %d !!!",bytesSent);
        RCLCPP_INFO(get_logger(),"sender linear.x:  %f !!!",msg->linear.x);

        if (bytesSent == -1) {
            perror("Error sending message");
            RCLCPP_INFO(get_logger(),"sender linear.x error %d !!!",bytesSent);
            std::exit(1);
        }

        complex_cmd_.cmd_code =325;
        complex_cmd_.size = 8;
        complex_cmd_.type = 1;
        complex_cmd_.data = msg->linear.y;                 ///< Lateral velocity
        ssize_t bytesSent_1 = sendto(socked_fd_, &complex_cmd_, sizeof(complex_cmd_), 0,
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytesSent_1 == -1) {
            perror("Error sending message");
            RCLCPP_INFO(get_logger(),"sender linear.y error %d !!!",bytesSent_1);
            std::exit(1);
        }

        complex_cmd_.cmd_code = 321;
        complex_cmd_.size = 8;
        complex_cmd_.type = 1;
        complex_cmd_.data = -1 * msg->angular.z;               ///< angular velocity
        ssize_t bytesSent_2 = sendto(socked_fd_, &complex_cmd_, sizeof(complex_cmd_), 0,
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytesSent_2 == -1) {
            perror("Error sending message");
            RCLCPP_INFO(get_logger(),"sender angular.z error %d !!!",bytesSent_2);
            std::exit(1);
        }
    };
};
int main(int argc,char**argv) {
 if(argc==2 && std::string(argv[1])=="dwb") {
  dwb_plugins::OneDVelocityIterator theta(0,-.4,.4,.5,-.5,2.5,31);
  while(!theta.isFinished()) {double w=theta.getVelocity();
   // Foxy XYThetaIterator joint minimum rule, x=y=0 for this test.
   if (!(0.0+1e-5<.03*.03 && std::abs(w)+1e-5<.1) && w!=0) std::cout<<std::setprecision(17)<<w<<"\n";
   ++theta;
  }
  return 0;
 }
 if(argc!=4) return 2;
 auto m=std::make_shared<geometry_msgs::msg::Twist>();
 m->linear.x=std::stod(argv[1]);m->linear.y=std::stod(argv[2]);m->angular.z=std::stod(argv[3]);
 MockSender sender;sender.CmdVelCallback(m);
 for(const auto&p:packets) {for(auto v:p) std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(v);std::cout<<"\n";}
}
