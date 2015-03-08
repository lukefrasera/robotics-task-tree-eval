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
#include <vector>
#include <cstdint>
#include <string>
namespace task_net {


typedef NodeId_t string;

struct NodeBitmask {
  uint8_t type;
  uint8_t robot;
  uint16_t node;
};

struct State {
  NodeId_t owner;  // If owener is null node is inactive
  bool active;
  bool done;
};

typedef std::vector<NodeId_t> NodeList;
typedef std::vector<ros::Publisher> PubList;

/*
Class: Node
Definition: Base class for behavior network nodes. All nodes will inherit from
            this class. This list includes AND, THEN, OR, WHILE nodes.
Author: Luke Fraser
*/
class Node {
 public:
  Node();
  Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent);
  virtual ~Node();

 protected:
  virtual void Activate();
  virtual void Deactivate();
  virtual void ActivateNode(NodeId_t node);
  virtual void DeactivateNode(NodeId_t node);
  virtual void Finish();
  virtual State GetState();

  // Messaging
  virtual void SendToParent(Msg message);
  virtual void SendToChild(NodeId_t node, Msg message);
  virtual void SendToPeer(NodeId_t node, Msg message);

  // Receiving Threads
  virtual void ReceiveFromParent();
  virtual void ReceiveFromChildred();
  virtual void ReceiveFromPeers();

  // Main Node loop functions
  virtual void NodeInit();
  virtual uint32_t IsDone();
  virtual float ActivationLevel();
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();

 private:
  virtual void InitalizeSubscriber(NodeId_t topic);
  virtual void InitalizePublishers(NodeList topics);
  virtual NodeBitmask GetBitmask(NodeId_t name);

  State state_;
  NodeId_t name_;
  NodeBitmask mask_;
  NodeList peers_;
  NodeList children_;
  NodeId_t parent_;

  // Publishers
  PubList children_pub_list_;
  PubList peer_pub_list_;

  // Subscribers
  ros::Subscriber children_sub_;
  ros::Subscriber peer_sub_;

  // Node handler
  ros::NodeHandle nh_;
};
}  // namespace task_net
#endif  // INCLUDE_NODE_H_
