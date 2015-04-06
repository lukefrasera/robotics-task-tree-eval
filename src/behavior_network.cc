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

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <signal.h>
#include <vector>
#include <string>
#include <map>
#include "../include/node.h"
#include "node_types.h"

void EndingFunc(int signal) {
  printf("Closing Program...\n");
  exit(0);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "behavior_network", ros::init_options::NoSigintHandler);
  signal(SIGINT, EndingFunc);
  ros::NodeHandle nh("~");

  std::string name_param;
  std::vector<std::string> peers_param;
  std::vector<std::string> children_param;
  std::string parent_param;

  if (nh.getParam("name", name_param)) {
    printf("%s\n", name_param.c_str());
  }
  if (nh.getParam("peers", peers_param)) {
    printf("%s\n", peers_param[0].c_str());
  }
  if (nh.getParam("children", children_param)) {
    printf("%s\n", children_param[0].c_str());
  }
  if (nh.getParam("parent", parent_param)) {
    printf("%s\n", parent_param.c_str());
  }

  task_net::Node * test;
  test = new task_net::Node(name_param,
                            peers_param,
                            children_param,
                            parent_param,
                            false);
  ros::spin();
  delete test;
  return 0;
}
