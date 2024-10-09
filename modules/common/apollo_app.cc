/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/apollo_app.h"

#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/status/status.h"
#include "modules/common/util/string_util.h"

#include "ros/include/ros/ros.h"

DECLARE_string(log_dir);

namespace apollo {
namespace common {

void ApolloApp::SetCallbackThreadNumber(uint32_t callback_thread_num) {
  CHECK_GE(callback_thread_num, 1);
  callback_thread_num_ = callback_thread_num;
}

void ApolloApp::ExportFlags() const {
  const auto export_file = util::StrCat(FLAGS_log_dir, "/", Name(), ".flags");
  std::ofstream fout(export_file);
  CHECK(fout) << "Cannot open file " << export_file;

  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);
  for (const auto& flag : flags) {
    fout << "# " << flag.type << ", default=" << flag.default_value << "\n"
         << "# " << flag.description << "\n"
         << "--" << flag.name << "=" << flag.current_value << "\n"
         << std::endl;
  }
}

int ApolloApp::Spin() {
  // 初始化应用程序，获取状态
  auto status = Init();
  if (!status.ok()) {
    // 如果初始化失败，记录错误并返回-1
    AERROR << Name() << " Init failed: " << status;
    return -1;
  }

  std::unique_ptr<ros::AsyncSpinner> spinner;
  // 如果需要多个回调线程，则创建AsyncSpinner
  if (callback_thread_num_ > 1) {
    spinner = std::unique_ptr<ros::AsyncSpinner>(
        new ros::AsyncSpinner(callback_thread_num_));
  }

  // 启动应用程序，获取状态
  status = Start();
  if (!status.ok()) {
    // 如果启动失败，记录错误并返回-2
    AERROR << Name() << " Start failed: " << status;
    return -2;
  }

  // 导出当前标志设置
  ExportFlags();
  
  // 启动异步线程或调用ros::spin()进行阻塞等待
  if (spinner) {
    spinner->start();
  } else {
    ros::spin();
  }

  // 等待ROS系统关闭
  ros::waitForShutdown();
  
  // 停止应用程序
  Stop();
  AINFO << Name() << " exited."; // 记录退出信息
  return 0; // 返回0表示正常退出
}

void apollo_app_sigint_handler(int signal_num) {
  AINFO << "Received signal: " << signal_num;
  if (signal_num != SIGINT) {
    return;
  }
  bool static is_stopping = false;
  if (is_stopping) {
    return;
  }
  is_stopping = true;
  ros::shutdown();
}

}  // namespace common
}  // namespace apollo
