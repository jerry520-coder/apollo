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

/**
 * @file
 */

#ifndef MODULES_COMMON_APOLLO_APP_H_
#define MODULES_COMMON_APOLLO_APP_H_

#include <csignal>
#include <string>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/status/status.h"

#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class ApolloApp
 *
 * @brief The base module class to define the interface of an Apollo app.
 * An Apollo app runs infinitely until being shutdown by SIGINT or ROS. Many
 * essential components in Apollo, such as localization and control are examples
 * of Apollo apps. The APOLLO_MAIN macro helps developer to setup glog, gflag
 * and ROS in one line.
 */
class ApolloApp {
 public:
  /**
   * @brief module name. It is used to uniquely identify the app.
   */
  virtual std::string Name() const = 0;

  /**
   * @brief this is the entry point of an Apollo App. It initializes the app,
   * starts the app, and stop the app when the ros has shutdown.
   */
  virtual int Spin();

  /**
   * The default destructor.
   */
  virtual ~ApolloApp() = default;

  /**
   * @brief set the number of threads to handle ros message callbacks.
   * The default thread number is 1
   */
  void SetCallbackThreadNumber(uint32_t callback_thread_num);

 protected:
  /**
   * @brief The module initialization function. This is the first function being
   * called when the App starts. Usually this function loads the configurations,
   * subscribe the data from sensors or other modules.
   * @return Status initialization status
   */

  /**
 * @brief 模块初始化函数。该函数是在应用启动时第一个被调用的函数。通常，该函数加载配置，订阅来自传感器或其他模块的数据。
 * @return 状态 初始化状态
 */
  virtual apollo::common::Status Init() = 0;

  /**
   * @brief The module start function. Apollo app usually triggered to execute
   * in two ways: 1. Triggered by upstream messages, or 2. Triggered by timer.
   * If an app is triggered by upstream messages, the Start() function usually
   * register a call back function that will be called when an upstream message
   * is received. If an app is triggered by timer, the Start() function usually
   * register a timer callback function.
   * @return Status start status
   */

  /**
 * @brief 模块启动函数。Apollo应用程序通常有两种方式被触发执行：1. 被上游消息触发，或 2. 被定时器触发。
 * 如果应用程序是由上游消息触发，Start()函数通常会注册一个回调函数，当接收到上游消息时会调用该函数。
 * 如果应用程序是由定时器触发，Start()函数通常会注册一个定时器回调函数。
 * @return 状态 启动状态
 */
  virtual apollo::common::Status Start() = 0;

  /**
   * @brief The module stop function. This function will be called when
   * after ros::shutdown() has finished. In the default APOLLO_MAIN macro,
   * ros::shutdown() is called when SIGINT is received.
   */

  /**
 * @brief 模块停止函数。该函数将在ros::shutdown()完成后被调用。在默认的APOLLO_MAIN宏中，当接收到SIGINT信号时会调用ros::shutdown()。
 */
  virtual void Stop() = 0;

  /** The callback thread number
   */
  uint32_t callback_thread_num_ = 1;

 private:
  /**
   * @brief Export flag values to <FLAGS_log_dir>/<name>.flags.
   */
  void ExportFlags() const;
};

void apollo_app_sigint_handler(int signal_num);

}  // namespace common
}  // namespace apollo

/**
 * @brief 
 *   
    // 初始化Google日志库，argv[0]是程序的名称
    google::InitGoogleLogging(argv[0]);                        \
    
    // 解析命令行参数
    google::ParseCommandLineFlags(&argc, &argv, true);         \
    
    // 捕获SIGINT信号（通常是Ctrl+C），并调用自定义的信号处理函数
    signal(SIGINT, apollo::common::apollo_app_sigint_handler); \
    
    // 创建APP类型的应用实例
    APP apollo_app_;                                           \
    
    // 初始化ROS系统，使用应用的名称
    ros::init(argc, argv, apollo_app_.Name());                 \
    
    // 运行应用的主循环
    apollo_app_.Spin();                                        \
 */
#define APOLLO_MAIN(APP)                                       \
  int main(int argc, char **argv) {                            \
    google::InitGoogleLogging(argv[0]);                        \
    google::ParseCommandLineFlags(&argc, &argv, true);         \
    signal(SIGINT, apollo::common::apollo_app_sigint_handler); \
    APP apollo_app_;                                           \
    ros::init(argc, argv, apollo_app_.Name());                 \
    apollo_app_.Spin();                                        \
    return 0;                                                  \
  }

#endif  // MODULES_COMMON_APOLLO_APP_H_
