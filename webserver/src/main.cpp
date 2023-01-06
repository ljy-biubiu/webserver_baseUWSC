// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "uWS.h"
#include "websocket.h"
#include "rosTalk/rostalk.h"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<websocket::Websocket> web_talk =
      std::make_shared<websocket::Websocket>();
  rclcpp::spin(std::make_shared<RosTalk>(web_talk));
  rclcpp::shutdown();
}




// //不能删，没有这个会报缺少文件
// void test(uWS::Hub h, std::atomic<int> expectedRequests)
// {
//   auto controlData = [&h, &expectedRequests](uWS::HttpResponse *res, char *data, size_t length, size_t remainingBytes)
//   {
//     res->end();
//   };
//   h.onHttpData([&controlData](uWS::HttpResponse *res, char *data, size_t length, size_t remainingBytes)
//                { controlData(res, data, length, remainingBytes); });
// }
