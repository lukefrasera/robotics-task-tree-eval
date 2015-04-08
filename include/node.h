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
#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <stdint.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <map>
#include "node_types.h"
#include "robotics_task_tree_eval/ControlMessage.h"

namespace task_net {

typedef boost::shared_ptr<robotics_task_tree_eval::ControlMessage const>
  ConstControlMessagePtr;
typedef boost::shared_ptr<robotics_task_tree_eval::ControlMessage>
  ControlMessagePtr;
/*
Class: Node
Definition: Base class for behavior network nodes. All nodes will inherit from
            this class. This list includes AND, THEN, OR, WHILE nodes.
Author: Luke Fraser
*/
class Node {
 public:
  Node();
  Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(1000));
  virtual ~Node();

  virtual void Update();

 protected:
  virtual void Activate();
  virtual void Deactivate();
  virtual void ActivateNode(NodeId_t node);
  virtual void DeactivateNode(NodeId_t node);
  virtual void Finish();
  virtual State GetState();

  // Messaging
  virtual void SendToParent(
    const robotics_task_tree_eval::ControlMessage msg);
  virtual void SendToChild(NodeBitmask node,
    const robotics_task_tree_eval::ControlMessage msg);
  virtual void SendToPeer(NodeBitmask node,
    const robotics_task_tree_eval::ControlMessage msg);

  // Receiving Threads
  virtual void ReceiveFromParent(ConstControlMessagePtr msg);
  virtual void ReceiveFromChildren(ConstControlMessagePtr msg);
  virtual void ReceiveFromPeers(ConstControlMessagePtr msg);

  // Main Node loop functions
  virtual bool IsDone();
  virtual bool IsActive();
  virtual float ActivationLevel();
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
  virtual ros::CallbackQueue* GetPubCallbackQueue();
  virtual ros::CallbackQueue* GetSubCallbackQueue();

  ros::CallbackQueue *pub_callback_queue_;
  ros::CallbackQueue *sub_callback_queue_;

 private:
  virtual void NodeInit(boost::posix_time::millisec mtime);
  virtual void PublishStatus();
  virtual void InitializeSubscriber(NodeId_t node);
  virtual void InitializePublishers(NodeList nodes, PubList *pub);
  virtual void InitializePublisher(NodeId_t node, ros::Publisher *pub);
  virtual void InitializeStatePublisher(NodeId_t node, ros::Publisher *pub);
  virtual NodeBitmask GetBitmask(NodeId_t name);
  virtual NodeId_t GetNodeId(NodeBitmask id);
  // virtual void GenerateNodeBitmaskMap();

  State state_;
  NodeId_t name_;
  std::map<NodeBitmask, NodeId_t, BitmaskLessThan> node_dict_;
  std::string name_id_;
  NodeBitmask mask_;
  NodeList peers_;
  NodeList children_;
  NodeId_t parent_;

  // Publishers
  PubList children_pub_list_;
  PubList peer_pub_list_;
  ros::Publisher parent_pub_;
  ros::Publisher self_pub_;

  // Subscribers
  ros::Subscriber children_sub_;
  ros::Subscriber peer_sub_;

  // Node handler
  ros::NodeHandle pub_nh_;
  ros::NodeHandle sub_nh_;

  // Threads
  boost::thread *update_thread;
};
}  // namespace task_net
#endif  // INCLUDE_NODE_H_
