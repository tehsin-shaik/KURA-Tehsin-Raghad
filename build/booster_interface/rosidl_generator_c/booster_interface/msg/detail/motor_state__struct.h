// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from booster_interface:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef BOOSTER_INTERFACE__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define BOOSTER_INTERFACE__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorState in the package booster_interface.
typedef struct booster_interface__msg__MotorState
{
  int8_t mode;
  float q;
  float dq;
  float ddq;
  float tau_est;
  /// 电机温度信息：类型：int8_t ，可按照实际数值显示（范围：-100 - 150）。
  int8_t temperature;
  /// 电机丢包信息：可按照实际数值显示（范围：0-9999999999）。
  uint32_t lost;
  /// 当前电机通信频率+电机错误标志位：（数组：0-电机错误标志位（范围：0-255，可按照实际数值显示），1-当前电机通信频率（范围：0-800，可按照实际数值显示））
  uint32_t reserve[2];
} booster_interface__msg__MotorState;

// Struct for a sequence of booster_interface__msg__MotorState.
typedef struct booster_interface__msg__MotorState__Sequence
{
  booster_interface__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} booster_interface__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BOOSTER_INTERFACE__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
