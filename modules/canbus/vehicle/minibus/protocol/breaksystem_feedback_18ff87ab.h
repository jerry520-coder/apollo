/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/canbus/proto/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

class Breaksystemfeedback18ff87ab
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Breaksystemfeedback18ff87ab();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  double brk_fb_systemdefault(const std::uint8_t* bytes,
                              const int32_t length) const;

  double epb_fb_main_brkpressure(const std::uint8_t* bytes,
                                 const int32_t length) const;

  double epb_fb_right_brkpressure_setvaul(const std::uint8_t* bytes,
                                          const int32_t length) const;

  double epb_fb_left_brkpressure_setvaule(const std::uint8_t* bytes,
                                          const int32_t length) const;

  bool brk_fb_epb_feedback(const std::uint8_t* bytes,
                           const int32_t length) const;

  bool bek_fb_break_enable(const std::uint8_t* bytes,
                           const int32_t length) const;

  bool brk_fb_overhot_warning(const std::uint8_t* bytes,
                              const int32_t length) const;

  double brk_fb_right_breakpressure(const std::uint8_t* bytes,
                                    const int32_t length) const;

  double brk_fb_left_breakpressure(const std::uint8_t* bytes,
                                   const int32_t length) const;
};

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo