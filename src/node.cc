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
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <stdlib.h>
#include <string>
#include <vector>
#include "robotics_task_tree_eval/State.h"

namespace task_net {

#define PUB_SUB_QUEUE_SIZE 100
#define STATE_MSG_LEN (sizeof(State))

Node::Node() {
  state_.active = false;
  state_.done = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
  bool use_local_callback_queue, boost::posix_time::millisec mtime) {
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

  state_.owner.type = 1;
  state_.owner.robot = 0;
  state_.owner.node = 0;
  state_.active = true;
  state_.done = true;
  state_.activation_level = 0.0f;

  // Get bitmask
  mask_ = GetBitmask(name_);
  // Generate reverse map
  // GenerateNodeBitmaskMap();
  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_, &children_pub_list_);
  InitializePublishers(peers_, &peer_pub_list_);
  InitializePublisher(parent_, &parent_pub_);
  InitializeStatePublisher(name_, &self_pub_);
  NodeInit(mtime);
}

Node::~Node() {}

// void Node::GenerateNodeBitmaskMap() {
//   std::vector<std::string> nodes;
//   if (pub_nh_.getParam("Nodes", nodes)) {
//     printf("Generating BitmaskMap\n");
//     for (std::vector<std::string>::iterator it = nodes.begin();
//       it != nodes.end(); ++it) {
//       node_dict_[GetBitmask(*it)] = *it;
//     }
//   }
// }
void Node::Activate() {
  // if (!state_.active) {
  //   state_.active = true;
  //   printf("Activating Node: %s\n", name_.c_str());
  // }
}

void Node::Deactivate() {
  // if (state_.active && state_.owner == name_.c_str()) {
  //   state_.active = false;
  //   printf("Deactivating Node; %s\n", name_.c_str());
  // }
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

void Node::SendToParent(const robotics_task_tree_eval::ControlMessage msg) {
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  parent_pub_.publish(msg_temp);
}
void Node::SendToChild(NodeBitmask node,
  const robotics_task_tree_eval::ControlMessage msg) {
  // get publisher for specific node
  ros::Publisher* pub = &node_dict_[node];
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}
void Node::SendToPeer(NodeBitmask node,
  const robotics_task_tree_eval::ControlMessage msg) {
  // get publisher for specific node
  ros::Publisher* pub = &node_dict_[node];
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}

void Node::ReceiveFromParent(ConstControlMessagePtr msg) {
  // Set activation level from parent
  // TODO(Luke Fraser) Use mutex to avoid race condition
  state_.activation_level = msg->activation_level;
}
void Node::ReceiveFromChildren(ConstControlMessagePtr msg) {}
void Node::ReceiveFromPeers(ConstControlMessagePtr msg) {}


// Main Loop of Update Thread. spins once every mtime milliseconds
void UpdateThread(Node *node, boost::posix_time::millisec mtime) {
  while (true) {
    node->Update();
    boost::this_thread::sleep(mtime);
  }
}

// Initialize node threads and variables
void Node::NodeInit(boost::posix_time::millisec mtime) {
  // Initialize node threads
  update_thread = new boost::thread(&UpdateThread, this, mtime);
}

// Main Loop of the Node type Each Node Will have this fucnction called at each
// times step to process node properties. Each node should run in its own thread
void Node::Update() {
  // Check if Done
  if (!IsDone()) {
    // Check Activation Level
    if (IsActive()) {
      // Check Preconditions
      if (Precondition()) {
        // Print temp
        printf("Preconditions Satisfied Safe To Do Work!\n");
      } else {
        printf("Preconditions Not Satisfied, Spreading Activation!\n");
        SpreadActivation();
      }
      // Do Work
    }
  }
  // Publish Status
  PublishStatus();
}

// Deprecated function. use ros message data type with struct generality.
std::string StateToString(State state) {
  char buffer[sizeof(State)*8];
  snprintf(buffer, sizeof(buffer), "Owner:%u, Actvie:%d, Done:%d, Level:%f",
    *reinterpret_cast<uint32_t*>(&state),
    *(reinterpret_cast<uint8_t*>(&state)+sizeof(NodeBitmask)),
    *(reinterpret_cast<uint8_t*>(&state)+sizeof(NodeBitmask)+sizeof(bool)),
    *(reinterpret_cast<float*>(&state)+sizeof(NodeBitmask)+sizeof(bool)*2));
  std::string str = buffer;
  return str;
}

void Node::PublishStatus() {
  boost::shared_ptr<State_t> msg(new State_t);
  *msg = state_;
  self_pub_.publish(msg);
  // printf("Publish Status: %s\n", msg->data.c_str());
}

bool Node::IsDone() {
  return state_.done;
}
bool Node::IsActive() {
  return state_.active;
}
float Node::ActivationLevel() {}
bool Node::Precondition() {
  return false;
}
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
    ros::Publisher topic =
      pub_nh_.advertise<robotics_task_tree_eval::ControlMessage>(*it,
        PUB_SUB_QUEUE_SIZE);

    pub->push_back(topic);
    node_dict_[GetBitmask(*it)] = topic;
#if DEBUG
    printf("[PUBLISHER] - Creating Topic: %s\n", it->c_str());
#endif
  }
}

void Node::InitializePublisher(NodeId_t topic, ros::Publisher *pub) {
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", topic.c_str());
#endif
  (*pub) =
    pub_nh_.advertise<robotics_task_tree_eval::ControlMessage>(topic,
      PUB_SUB_QUEUE_SIZE);
  node_dict_[GetBitmask(topic)] = *pub;
}

void Node::InitializeStatePublisher(NodeId_t topic, ros::Publisher *pub) {
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", topic.c_str());
#endif
  (*pub) = pub_nh_.advertise<robotics_task_tree_eval::State>(topic,
    PUB_SUB_QUEUE_SIZE);
  node_dict_[GetBitmask(topic)] = *pub;
}

NodeBitmask Node::GetBitmask(NodeId_t name) {
  // Split underscores
  std::vector<std::string> split_vec;
  boost::algorithm::split(split_vec, name,
    boost::algorithm::is_any_of("_"));
  NodeBitmask mask;
  // node_type
  mask.type  = static_cast<uint8_t>(atoi(split_vec[1].c_str()));
  mask.robot = static_cast<uint8_t>(atoi(split_vec[2].c_str()));
  mask.node  = static_cast<uint16_t>(atoi(split_vec[3].c_str()));
  return mask;
}
NodeId_t Node::GetNodeId(NodeBitmask id) {
  NodeId_t nid;
  char buffer[50];
  nid = name_id_;
  nid += "_";
  snprintf(buffer, sizeof(buffer), "%d", id.type);
  nid += buffer;
  nid += "_";
  snprintf(buffer, sizeof(buffer), "%d", id.robot);
  nid += "_";
  snprintf(buffer, sizeof(buffer), "%d", id.node);
  nid += buffer;
  return nid;
}

ros::CallbackQueue* Node::GetPubCallbackQueue() {
  return pub_callback_queue_;
}
ros::CallbackQueue* Node::GetSubCallbackQueue() {
  return sub_callback_queue_;
}
}  // namespace task_net
