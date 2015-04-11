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
#define ACTIVATION_THESH 0.1
#define ACTIVATION_FALLOFF 0.95f

Node::Node() {
  state_.active = false;
  state_.done = false;
}

Node::Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
  State_t state,
  bool use_local_callback_queue, boost::posix_time::millisec mtime) {
  if (use_local_callback_queue) {
  #ifdef DEBUG
    printf("Local Callback Queues\n");
  #endif
    pub_nh_.setCallbackQueue(pub_callback_queue_);
    sub_nh_.setCallbackQueue(sub_callback_queue_);
  }
  // Generate reverse map
  GenerateNodeBitmaskMap();

  name_   = node_dict_[GetBitmask(name.topic)];
  for (NodeListIterator it = peers.begin(); it != peers.end(); ++it) {
    peers_.push_back(node_dict_[GetBitmask(it->topic)]);
  }
  for (NodeListIterator it = children.begin(); it != children.end(); ++it) {
    children_.push_back(node_dict_[GetBitmask(it->topic)]);
  }
  parent_ = node_dict_[GetBitmask(parent.topic)];

  // Setup bitmasks
  InitializeBitmask(name_);
  InitializeBitmasks(peers_);
  InitializeBitmasks(children_);
  InitializeBitmask(parent_);

  state_ = state;
  state_.owner = name_->mask;
  state_.active = false;
  state_.done = false;
  state_.activation_level = 0.0f;

  // Get bitmask
  printf("name: %s\n", name_->topic.c_str());
  mask_ = GetBitmask(name_->topic);
  // Setup Publisher/subscribers
  InitializeSubscriber(name_);
  InitializePublishers(children_, &children_pub_list_, "_parent");
  InitializePublishers(peers_, &peer_pub_list_, "_peer");
  InitializePublisher(parent_, &parent_pub_);
  InitializeStatePublisher(name_, &self_pub_, "_state");
  NodeInit(mtime);
}

Node::~Node() {}

void Node::InitializeBitmask(NodeId_t * node) {
  node->mask = GetBitmask(node->topic);
}

void Node::InitializeBitmasks(NodeListPtr nodes) {
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    InitializeBitmask(*it);
  }
}

void Node::GenerateNodeBitmaskMap() {
  std::vector<std::string> nodes;
  if (pub_nh_.getParam("Nodes", nodes)) {
    printf("Generating BitmaskMap\n");
    for (std::vector<std::string>::iterator it = nodes.begin();
        it != nodes.end(); ++it) {
      NodeId_t *nptr = new NodeId_t;
      nptr->topic = *it;
      nptr->mask = GetBitmask(*it);
      nptr->pub = NULL;
      nptr->state =  {nptr->mask, false, false, 0.0f};
      node_dict_[nptr->mask] = nptr;
      printf("Adding [%s] to Dictionary.\n", nptr->topic.c_str());
    }
  }
}
void Node::Activate() {
  if (!state_.active) {
    if (ActivationPrecondition()) {
      printf("Activating Node: %s\n", name_->topic.c_str());
      {
        boost::lock_guard<boost::mutex> lock(work_mut);
        state_.active = true;
      }
      cv.notify_one();
    }
  }
}

bool Node::ActivationPrecondition() {
  return true;
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
void Node::SendToParent(const ControlMessagePtr_t msg) {
  parent_pub_.publish(msg);
}
void Node::SendToChild(NodeBitmask node,
  const robotics_task_tree_eval::ControlMessage msg) {
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}
void Node::SendToChild(NodeBitmask node, const ControlMessagePtr_t msg) {
  node_dict_[node]->pub->publish(msg);
}
void Node::SendToPeer(NodeBitmask node,
  const robotics_task_tree_eval::ControlMessage msg) {
  // get publisher for specific node
  ros::Publisher* pub = node_dict_[node]->pub;
  // publish message to the specific child
  ControlMessagePtr msg_temp(new robotics_task_tree_eval::ControlMessage);
  *msg_temp = msg;
  pub->publish(msg_temp);
}
void Node::SendToPeer(NodeBitmask node, const ControlMessagePtr_t msg) {
  node_dict_[node]->pub->publish(msg);
}

void Node::ReceiveFromParent(ConstControlMessagePtr_t msg) {
  // Set activation level from parent
  // TODO(Luke Fraser) Use mutex to avoid race condition setup in publisher
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level = msg->activation_level;
}
void Node::ReceiveFromChildren(ConstControlMessagePtr_t msg) {
  // Determine the child
  NodeId_t *child = node_dict_[msg->sender];
  boost::unique_lock<boost::mutex> lck(mut);
  child->state.activation_level = msg->activation_level;
  child->state.done = msg->done;
}
void Node::ReceiveFromPeers(ConstControlMessagePtr_t msg) {
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level = msg->activation_level;
  state_.done = msg->done;
}

// Main Loop of Update Thread. spins once every mtime milliseconds
void UpdateThread(Node *node, boost::posix_time::millisec mtime) {
  while (true) {
    node->Update();
    boost::this_thread::sleep(mtime);
  }
}

void WorkThread(Node *node) {
  boost::unique_lock<boost::mutex> lock(node->work_mut);
  while (!node->state_.active) {
    node->cv.wait(lock);
  }
  // Process Data
  node->Work();
  boost::unique_lock<boost::mutex> lck(node->mut);
  node->state_.active = false;
  node->state_.done = true;
}
// Initialize node threads and variables
void Node::NodeInit(boost::posix_time::millisec mtime) {
  // Initialize node threads
  update_thread = new boost::thread(&UpdateThread, this, mtime);
  work_thread   = new boost::thread(&WorkThread, this);
}

void Node::ActivationFalloff() {
  boost::unique_lock<boost::mutex> lck(mut);
  state_.activation_level *= ACTIVATION_FALLOFF;
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
#ifdef DEBUG
        printf("Preconditions Satisfied Safe To Do Work!\n");
#endif
        Activate();
      } else {
#ifdef DEBUG
        printf("Preconditions Not Satisfied, Spreading Activation!\n");
#endif
        SpreadActivation();
      }
      ActivationFalloff();
    } else {
#ifdef DEBUG
      printf("Not Active: %f\n", state_.activation_level);
#endif
    }
  }
  // Publish Status
  PublishStatus();
}

void Node::Work() {
  printf("Doing Work\n");

  printf("Done!\n");
}
// Deprecated function. use ros message data type with struct generality.
std::string StateToString(State state) {
  char buffer[sizeof(State)*8];
  snprintf(buffer, sizeof(buffer), "Owner:%u, Active:%d, Done:%d, Level:%f",
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
  return state_.activation_level > ACTIVATION_THESH;
}
float Node::ActivationLevel() {
  return state_.activation_level;
}
bool Node::Precondition() {
  // TODO(Luke Fraser) Merge children/peer/name/parent lists to point to the same as dictionary
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}
uint32_t Node::SpreadActivation() {}
void Node::InitializeSubscriber(NodeId_t *node) {
  std::string peer_topic = node->topic + "_peers";
#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Peer Topic: %s\n", peer_topic.c_str());
#endif
  peer_sub_     = sub_nh_.subscribe(peer_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromPeers,
    this);

#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Child Topic: %s\n", node->topic.c_str());
#endif
  children_sub_ = sub_nh_.subscribe(node->topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromChildren,
    this);
  std::string parent_topic = node->topic + "_parent";
#ifdef DEBUG
  printf("[SUBSCRIBER] - Creating Parent Topic: %s\n", parent_topic.c_str());
#endif
  parent_sub_ = sub_nh_.subscribe(parent_topic,
    PUB_SUB_QUEUE_SIZE,
    &Node::ReceiveFromParent,
    this);
}
void Node::InitializePublishers(NodeListPtr nodes, PubList *pub,
    const char * topic_addition) {
  for (NodeListPtrIterator it = nodes.begin(); it != nodes.end(); ++it) {
    ros::Publisher * topic = new ros::Publisher;
    *topic =
      pub_nh_.advertise<robotics_task_tree_eval::ControlMessage>(
        (*it)->topic + topic_addition,
        PUB_SUB_QUEUE_SIZE);

    pub->push_back(*topic);
    node_dict_[(*it)->mask]->pub = topic;
    node_dict_[(*it)->mask]->topic += topic_addition;
#if DEBUG
    printf("[PUBLISHER] - Creating Topic: %s\n", (*it)->topic.c_str());
#endif
  }
}

void Node::InitializePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition) {
  node->topic += topic_addition;
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", node->topic.c_str());
#endif
  (*pub) =
    pub_nh_.advertise<robotics_task_tree_eval::ControlMessage>(node->topic,
      PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

void Node::InitializeStatePublisher(NodeId_t *node, ros::Publisher *pub,
  const char * topic_addition) {
  node->topic += topic_addition;
#ifdef DEBUG
  printf("[PUBLISHER] - Creating Topic: %s\n", node->topic.c_str());
#endif
  (*pub) = pub_nh_.advertise<robotics_task_tree_eval::State>(node->topic,
    PUB_SUB_QUEUE_SIZE);
  node_dict_[node->mask]->pub = pub;
  // node_dict_[node.mask]->topic += topic_addition;
}

NodeBitmask Node::GetBitmask(std::string name) {
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
  return *node_dict_[id];
}

ros::CallbackQueue* Node::GetPubCallbackQueue() {
  return pub_callback_queue_;
}
ros::CallbackQueue* Node::GetSubCallbackQueue() {
  return sub_callback_queue_;
}
}  // namespace task_net
