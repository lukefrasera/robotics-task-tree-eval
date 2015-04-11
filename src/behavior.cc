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
#include "../include/behavior.h"
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <vector>

namespace task_net {
typedef std::vector<NodeId_t>::iterator NodeId_t_iterator;
// BEHAVIOR
Behavior::Behavior() {}
Behavior::Behavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Node(name,
      peers,
      children,
      parent,
      state) {}
Behavior::~Behavior() {}

////////////////////////////////////////////////////////////////////////////////
// AND BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
AndBehavior::AndBehavior() {}
AndBehavior::AndBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {}
AndBehavior::~AndBehavior() {}

bool AndBehavior::Precondition() {
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t AndBehavior::SpreadActivation() {
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = 1.0f / children_.size();
  msg->done = false;

  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    SendToChild((*it)->mask, msg);
  }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// THEN BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
ThenBehavior::ThenBehavior() {}
ThenBehavior::ThenBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
  // Initialize activation queue
  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    activation_queue_.push(*it);
  }
}
ThenBehavior::~ThenBehavior() {}

bool ThenBehavior::Precondition() {
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t ThenBehavior::SpreadActivation() {
  if (!activation_queue_.empty()) {
    ControlMessagePtr_t msg(new ControlMessage_t);
    msg->sender = mask_;
    msg->activation_level = 1.0f;
    msg->done = false;

    if (activation_queue_.front()->state.done) {
      activation_queue_.pop();
    }

    SendToChild(activation_queue_.front()->mask, msg);
  }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// OR BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
OrBehavior::OrBehavior() {}
OrBehavior::OrBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
  seed = static_cast<uint32_t>(time(NULL));
  random_child_selection = rand_r(&seed) % children_.size();
}
OrBehavior::~OrBehavior() {}

bool OrBehavior::Precondition() {
  if (children_[random_child_selection]->state.done)
    return true;
  return false;
}

uint32_t OrBehavior::SpreadActivation() {
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->sender = mask_;
  msg->activation_level = 1.0f;
  msg->done = false;

  SendToChild(children_[random_child_selection]->mask, msg);
}
}  // namespace task_net
