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

#ifndef WEBSOCKET_INCLUDE_H_
#define WEBSOCKET_INCLUDE_H_

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <jsoncpp/json/json.h>

#include "server/uws_server.h"
#include "threads/threadpool.h"

namespace websocket
{

  enum CONTENT
  {
    CONTENT_NAME = 0,
    CONTENT_DATA = 1,
  };

  class Websocket
  {
  public:
    Websocket();
    ~Websocket();

    void pushData(const std::vector<std::string> &msg);

  private:
    std::queue<std::vector<std::string>> my_queue;

    std::shared_ptr<UwsServer> uws_server_;
    std::shared_ptr<std::thread> worker_;
    hobot::CThreadPool data_send_thread_;
    bool map_stop_ = false;
    std::mutex my_mutex;
    std::thread *sendMsg;

    int SendMessage(const std::vector<std::string> &msg);
    void MessageProcess(void);
    std::string packageJson(const std::string &name, const std::string &data);

    // 通过websocket协议对外输出数据的帧率控制，用于支持在wifi弱网络情况下web端流畅展示
    // 默认以15fps的速率输出
    int output_fps_ = 10;
    int send_frame_count_ = 0;
  };

} // namespace websocket

#endif