/*
robotics-task-tree-eval
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef NODE_TYPES_H_
#define NODE_TYPES_H_

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <stdint.h>
#include <string>
#include <vector>

namespace task_net {
typedef std::string NodeId_t;

struct NodeBitmask {
  uint8_t type;
  uint8_t robot;
  uint16_t node;
};

struct State {
  NodeBitmask owner;  // If owner is null node is inactive
  bool active;
  bool done;
  float activation_level;
};

typedef std::vector<NodeId_t> NodeList;
typedef std::vector<ros::Publisher> PubList;
typedef State State_t;

/*// State messages ros
ROS_STATIC_ASSERT(sizeof(State) == sizeof(State_t));

namespace ros {
namespace message_traits {
template<>
struct IsFixedSize<State> : public TrueType {};
template<>
struct IsSimple<State> : public TrueType {};

template<>
struct MD5Sum<State> {
  static const  char* value() {
    return "b1726aca0cd47f1e0dcb08bbf896cf72";
  }

  static const char* value(const State& m) {return value();}
};

template <>
struct DataType<State> {
  static const char* value() {
    return 0;
  }

  static const char* value(const State& m) {return value();}
};

template<>
struct Definition<State> {
  static const char* value() {
    return 0;
  }
  static const char* value(const State& m) {return value();}
};
}  // namespace message_traits

namespace serialization {
template <>
struct Serialization<State> {
  template<typename Stream, typename T>
  inline static void allInOne(Steam& stream, T t) {
    stream.next(t.owner);
    stream.next(t.active);
    stream.next(t.done);
    stream.next(t.activation_level);
  }
};
}  // namespace serialization
}  // namespace ros*/
}  // namespace task_net
#endif
