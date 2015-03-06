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
#include "node.h"
namespace task_net {

Node::Node() {
  state.owner = NULL;
  state.active = false;
  state.done = false;
}

Node::~Node() {}

void Node::Activate() {
  if (!state.active) {
    state.active = true;
    state.owner = this;
    printf("Activating Node: %d\n", static_cast<int>(this));
  }
}

void Node::Deactivate() {
  if (state.active && state.owner == this) {
    state.active = false;
    state.owner = NULL;
    printf("Deactivating Node; %d\n", static_cast<int>(this));
  }
}

void Node::ActivateNode(Node * node) {
  node.Activate();
}

void Node::DeactivateNode(Node * node) {
  node.Deactivate();
}

void Node::Finish() {
  Deactivate();
  state.done = true;
}

State Node::GetState() {
  return state;
}

void Node::SendToParent(Msg message) {}
void Node::SendToChild(Node* node, Msg message) {}
void Node::SendToPeer(Node* node, Msg message) {}

void Node::ReceiveFromParent(Msg message) {}
void Node::ReceiveFromChildren() {}
void Node::ReceiveFromPeers() {}

// Main Loop of the Node type Each Node Will have this fucnction called at each
// times step to process node properties. Each node should run in its own thread
void Node::NodeInit() {}
uint32_t Node::IsDone();
float Node::ActivationLevel() {}
bool Node::Precondition() {}
uint32_t Node::SpreadActivation() {}

}  // namespace task_net

