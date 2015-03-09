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
#include "../include/node.h"
namespace task_net {

Node::Node() {
  state_.owner = "";
  state_.active = false;
  state_.done = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent) {
  // Call base constructor to set state
  state_.owner = "";
  state_.active = false;
  state_.done = false;

  name_ = name;
  peers_    = peers;
  children_ = children;
  parent_   = parent;

  // Get bitmask
  mask_ = GetBitmask(name_);
  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_);
  InitializePublishers(peers_);
  // InitializePublishers(children);
  // TODO(Luke FRaser) need to setup parent publication. This method isn't good
}

Node::~Node() {}

void Node::Activate() {
  if (!state_.active) {
    state_.active = true;
    state_.owner = name_;
    printf("Activating Node: %s\n", name_.c_str());
  }
}

void Node::Deactivate() {
  if (state_.active && state_.owner == name_.c_str()) {
    state_.active = false;
    state_.owner = "";
    printf("Deactivating Node; %s\n", name_.c_str());
  }
}

void Node::ActivateNode(NodeId_t node) {}

void Node::DeactivateNode(NodeId_t node) {}

void Node::Finish() {
  Deactivate();
  state_.done = true;
}

State Node::GetState() {
  return state_;
}

void Node::SendToParent(std_msgs::String message) {}
void Node::SendToChild(NodeId_t node, std_msgs::String message) {}
void Node::SendToPeer(NodeId_t node, std_msgs::String message) {}

void Node::ReceiveFromParent(std_msgs::String message) {}
void Node::ReceiveFromChildren() {}
void Node::ReceiveFromPeers() {}

// Main Loop of the Node type Each Node Will have this fucnction called at each
// times step to process node properties. Each node should run in its own thread
void Node::NodeInit() {}
uint32_t Node::IsDone() {}
float Node::ActivationLevel() {}
bool Node::Precondition() {}
uint32_t Node::SpreadActivation() {}
void Node::InitializeSubscriber(NodeId_t topic) {}
void Node::InitializePublishers(NodeList topics) {}
NodeBitmask Node::GetBitmask(NodeId_t name) {}
}  // namespace task_net
