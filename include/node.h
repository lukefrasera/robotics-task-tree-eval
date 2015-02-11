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
namespace task_net {
class Node;

struct State {
  Node * owner; // If owener is null node is inactive
  bool active;
  bool done;
};

typedef std::vector<Node> NodeList;
/*
Class: Node
Definition: Base class for behavior network nodes. All nodes will inherit from
            this class. This list includes AND, THEN, OR, WHILE nodes.
Author: Luke Fraser
*/
class Node {
 public:
  Node();
  virtual ~Node();
 protected:
  virtual void Activate();
  virtual void Deactivate();
  virtual void ActivateNode(Node *);
  virtual void DeactivateNode(Node *);
  virtual void Finish();
  virtual State GetState();
 private:
  State state;
  NodeList peer_list;
  NodeList children;
  Node * parent;
};
}  // namespace task_net
#endif  // INCLUDE_NODE_H_