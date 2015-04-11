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
#ifndef INCLUDE_BEHAVIOR_H_
#define INCLUDE_BEHAVIOR_H_

#include "node.h"
namespace task_net {
class Behavior: public Node {
 public:
  Behavior();
  Behavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(1000));
  virtual ~Behavior();

 private:
};

class ThenBehavior: public Behavior {
 public:
  ThenBehavior();
  ThenBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(1000));
  virtual ~ThenBehavior();
 protected:
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
};
class AndBehavior: public Behavior {};
class OrBehavior: public Behavior {};
class WhileBehavior: public Behavior {};
}  // namespace task_net
#endif  // INCLUDE_BEHAVIOR_H_
