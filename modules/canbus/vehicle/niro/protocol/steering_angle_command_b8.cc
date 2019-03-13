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

#include "modules/canbus/vehicle/niro/protocol/steering_angle_command_b8.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace niro {

using ::apollo::drivers::canbus::Byte;

const int32_t Steeringanglecommandb8::ID = 0xB8;

// public
Steeringanglecommandb8::Steeringanglecommandb8() { Reset(); }

uint32_t Steeringanglecommandb8::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Steeringanglecommandb8::UpdateData(uint8_t* data) {
  set_p_steering_angle_cmd_flags(data, steering_angle_cmd_flags_);
  set_p_steering_angle_cmd_angle(data, steering_angle_cmd_angle_);
  set_p_steering_angle_cmd_max_velocity(data, steering_angle_cmd_max_velocity_);
}

void Steeringanglecommandb8::Reset() {
  steering_angle_cmd_flags_ = 1;
  steering_angle_cmd_angle_ = 0.0;
  steering_angle_cmd_max_velocity_ = 100.0;
}

Steeringanglecommandb8* Steeringanglecommandb8::set_steering_angle_cmd_flags(
   char steering_angle_cmd_flags) {
  steering_angle_cmd_flags_ = steering_angle_cmd_flags;
  return this;
 }

// config detail: {'name': 'steering_angle_cmd_flags', 'enum': {0: 'STEERING_ANGLE_CMD_FLAGS_DISABLE', 1: 'STEERING_ANGLE_CMD_FLAGS_ENABLE'}, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|0]', 'bit': 0, 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Steeringanglecommandb8::set_p_steering_angle_cmd_flags(uint8_t* data,
    char steering_angle_cmd_flags) {
  int x = steering_angle_cmd_flags;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Steeringanglecommandb8* Steeringanglecommandb8::set_steering_angle_cmd_angle(
    double steering_angle_cmd_angle) {
  steering_angle_cmd_angle_ = steering_angle_cmd_angle;
  return this;
 }

// config detail: {'name': 'steering_angle_cmd_angle', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Steeringanglecommandb8::set_p_steering_angle_cmd_angle(uint8_t* data,
    double steering_angle_cmd_angle) {
  steering_angle_cmd_angle = ProtocolData::BoundedValue(-500.0, 500.0, steering_angle_cmd_angle);
  int x = steering_angle_cmd_angle / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 1);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 2);
  to_set1.set_value(t, 0, 8);
}


Steeringanglecommandb8* Steeringanglecommandb8::set_steering_angle_cmd_max_velocity(
    double steering_angle_cmd_max_velocity) {
  steering_angle_cmd_max_velocity_ = steering_angle_cmd_max_velocity;
  return this;
 }

// config detail: {'name': 'steering_angle_cmd_max_velocity', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 24, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Steeringanglecommandb8::set_p_steering_angle_cmd_max_velocity(uint8_t* data,
    double steering_angle_cmd_max_velocity) {
  steering_angle_cmd_max_velocity = ProtocolData::BoundedValue(0.0, 200.0, steering_angle_cmd_max_velocity);
  int x = steering_angle_cmd_max_velocity / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 3);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 4);
  to_set1.set_value(t, 0, 8);
}

}  // namespace niro
}  // namespace canbus
}  // namespace apollo
