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
#include <string>
#include <vector>
namespace task_net {

#define PUB_SUB_QUEUE_SIZE 100

Node::Node() {
  state_.owner = "";
  state_.active = false;
  state_.done = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
  bool use_local_callback_queue) {
  if (use_local_callback_queue) {
  #ifdef DEBUG
    printf("Local Callback Queues\n");
  #endif
    pub_nh_.setCallbackQueue(pub_callback_queue_);
    sub_nh_.setCallbackQueue(sub_callback_queue_);
  }

  name_     = name;
  peers_    = peers;
  children_ = children;
  parent_   = parent;

  // Get bitmask
  mask_ = GetBitmask(name_);
  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_, &children_pub_list_);
  InitializePublishers(peers_, &peer_pub_list_);
  InitializePublisher(parent_, &parent_pub_);
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

void Node::SendToParent(std_msgs::String message) {
  parent_pub_.publish(message);
}
void Node::SendToChild(NodeBitmask node, std_msgs::String message) {
  // get publisher for specific node

  // publish message to the specific child
}
void Node::SendToPeer(NodeId_t node, std_msgs::String message) {
  // get publisher for specific node

  // publish to peer
}

void Node::ReceiveFromParent(std_msgs::String message) {}
void Node::ReceiveFromChildren(boost::shared_ptr<std_msgs::String const> msg) {}
void Node::ReceiveFromPeers(boost::shared_ptr<std_msgs::String const> msg) {}

// Main Loop of the Node type Each Node Will have this fucnction called at each
// times step to process node properties. Each node should run in its own thread
void Node::Update() {
  std::string hello = "hello";
  std_msgs::String msg;
  msg.data = hello;
  parent_pub_.publish(msg);
}
void Node::NodeInit() {
  // Initialize node threads
}  // Iinti shouwl intialize the nodes threads. Main function should only be
  //  used to setup callbackqueues and spinners
uint32_t Node::IsDone() {}
float Node::ActivationLevel() {}
bool Node::Precondition() {}
uint32_t Node::SpreadActivation() {}
void Node::InitializeSubscriber(NodeId_t topic) {
  std::string peer_topic = topic + "_peers";
#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Peer Topic: %s\n", peer_topic.c_str());
#endif
  peer_sub_     = sub_nh_.subscribe(peer_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromPeers,
    this);

#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Child Topic: %s\n", topic.c_str());
#endif
  children_sub_ = sub_nh_.subscribe(topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromChildren,
    this);
}
void Node::InitializePublishers(NodeList topics, PubList *pub) {
  for (std::vector<NodeId_t>::iterator it = topics.begin();
    it != topics.end();
    ++it) {
    ros::Publisher topic = pub_nh_.advertise<std_msgs::String>(*it,
      PUB_SUB_QUEUE_SIZE);

    pub->push_back(topic);
#if DEBUG
    printf("[PUBLISHER] - Creating Topic: %s\n", it->c_str());
#endif
  }
}

void Node::InitializePublisher(NodeId_t topic, ros::Publisher *pub) {
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", topic.c_str());
#endif
  (*pub) = pub_nh_.advertise<std_msgs::String>(topic, PUB_SUB_QUEUE_SIZE);
}

NodeBitmask Node::GetBitmask(NodeId_t name) {}
NodeId_t Node::GetNodeId(NodeBitmask id) {}

ros::CallbackQueue* Node::GetPubCallbackQueue() {
  return pub_callback_queue_;
}
ros::CallbackQueue* Node::GetSubCallbackQueue() {
  return sub_callback_queue_;
}
}  // namespace task_net
