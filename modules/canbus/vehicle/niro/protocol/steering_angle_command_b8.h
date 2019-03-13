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

#ifndef MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_STEERING_ANGLE_COMMAND_B8_H_
#define MODULES_CANBUS_VEHICLE_NIRO_PROTOCOL_STEERING_ANGLE_COMMAND_B8_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace niro {

class Steeringanglecommandb8 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Steeringanglecommandb8();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'steering_angle_cmd_flags', 'enum': {0: 'STEERING_ANGLE_CMD_FLAGS_DISABLE', 1: 'STEERING_ANGLE_CMD_FLAGS_ENABLE'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  Steeringanglecommandb8* set_steering_angle_cmd_flags(char steering_angle_cmd_flags);

  // config detail: {'name': 'steering_angle_cmd_angle', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  Steeringanglecommandb8* set_steering_angle_cmd_angle(double steering_angle_cmd_angle);

  // config detail: {'name': 'steering_angle_cmd_max_velocity', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 24, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  Steeringanglecommandb8* set_steering_angle_cmd_max_velocity(double steering_angle_cmd_max_velocity);

 private:

  // config detail: {'name': 'steering_angle_cmd_flags', 'enum': {0: 'STEERING_ANGLE_CMD_FLAGS_DISABLE', 1: 'STEERING_ANGLE_CMD_FLAGS_ENABLE'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
  void set_p_steering_angle_cmd_flags(uint8_t* data, char steering_angle_cmd_flags);

  // config detail: {'name': 'steering_angle_cmd_angle', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  void set_p_steering_angle_cmd_angle(uint8_t* data, double steering_angle_cmd_angle);

  // config detail: {'name': 'steering_angle_cmd_max_velocity', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 24, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  void set_p_steering_angle_cmd_max_velocity(uint8_t* data, double steering_angle_cmd_max_velocity);

 private:
  char steering_angle_cmd_flags_;
  double steering_angle_cmd_angle_;
  double steering_angle_cmd_max_velocity_;
};

}  // namespace niro
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_NIRO_PROTOCOL_STEERING_ANGLE_COMMAND_B8_H_
