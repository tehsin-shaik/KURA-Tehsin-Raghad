// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from game_controller_interface:msg/TeamInfo.idl
// generated code does not contain a copyright notice

#ifndef GAME_CONTROLLER_INTERFACE__MSG__DETAIL__TEAM_INFO__STRUCT_H_
#define GAME_CONTROLLER_INTERFACE__MSG__DETAIL__TEAM_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'coach_message'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'coach'
// Member 'players'
#include "game_controller_interface/msg/detail/robot_info__struct.h"

/// Struct defined in msg/TeamInfo in the package game_controller_interface.
/**
  * 跟 include/RoboCupGameControllData.h 中的 TeamInfo 保持一致的结构
  * 稍注意一下，coach_message 和 players 在这里定义为不定长数组，更符合语义
 */
typedef struct game_controller_interface__msg__TeamInfo
{
  /// unique team number
  uint8_t team_number;
  /// colour of the team
  uint8_t field_player_colour;
  /// team's score
  uint8_t score;
  /// penalty shot counter
  uint8_t penalty_shot;
  /// bits represent penalty shot success
  uint16_t single_shots;
  /// sequence number of the coach's message
  uint8_t coach_sequence;
  /// the coach's message to the team (length=253)
  rosidl_runtime_c__uint8__Sequence coach_message;
  game_controller_interface__msg__RobotInfo coach;
  /// the team's players (length=11)
  game_controller_interface__msg__RobotInfo__Sequence players;
} game_controller_interface__msg__TeamInfo;

// Struct for a sequence of game_controller_interface__msg__TeamInfo.
typedef struct game_controller_interface__msg__TeamInfo__Sequence
{
  game_controller_interface__msg__TeamInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} game_controller_interface__msg__TeamInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAME_CONTROLLER_INTERFACE__MSG__DETAIL__TEAM_INFO__STRUCT_H_
