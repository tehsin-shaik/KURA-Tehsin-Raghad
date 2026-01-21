// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from game_controller_interface:msg/RobotInfo.idl
// generated code does not contain a copyright notice

#ifndef GAME_CONTROLLER_INTERFACE__MSG__DETAIL__ROBOT_INFO__STRUCT_H_
#define GAME_CONTROLLER_INTERFACE__MSG__DETAIL__ROBOT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RobotInfo in the package game_controller_interface.
/**
  * 跟 include/RoboCupGameControllData.h 中的 RobotInfo 保持一致的结构
 */
typedef struct game_controller_interface__msg__RobotInfo
{
  /// penalty state of the player
  uint8_t penalty;
  /// estimate of time till unpenalised
  uint8_t secs_till_unpenalised;
  /// number of warnings
  uint8_t number_of_warnings;
  /// number of yellow cards
  uint8_t yellow_card_count;
  /// number of red cards
  uint8_t red_card_count;
  /// flags if robot is goal keeper
  bool goal_keeper;
} game_controller_interface__msg__RobotInfo;

// Struct for a sequence of game_controller_interface__msg__RobotInfo.
typedef struct game_controller_interface__msg__RobotInfo__Sequence
{
  game_controller_interface__msg__RobotInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} game_controller_interface__msg__RobotInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAME_CONTROLLER_INTERFACE__MSG__DETAIL__ROBOT_INFO__STRUCT_H_
