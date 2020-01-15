/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Jonatan Gines jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */

#include "graph_dialog_extractor/graph_dialog_extractor_node.h"
#include <string>
#include <list>

namespace graph_dialog_extractor
{
GraphDialogExtractor::GraphDialogExtractor():
  floorDF("elevator_current_floor.ask"),
  orderReadyDF("orderReady.ask"),
  confirmOrderDF("confirmOrder.ask"),
  startDF("bar_start.action"),
  orderDF("bar_order.ask"),
  helloDF("hello.action"),
  byeDF("bye.action"),
  confirmationDF("[[:print:]_]*.confirmation")
{}

std::string GraphDialogExtractor::saySplit(std::string str)
{
  std::string delimiter = ":";
  size_t token = 0;
  token = str.find(delimiter, 0);
  str.erase(0, str.find(delimiter) + delimiter.length());
  return str;
}

void GraphDialogExtractor::step()
{
  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string edge = it->get();
    if (edge.find("say:") != std::string::npos)
    {
      graph_.remove_edge(*it);
      std::string say_splited_edge = saySplit(edge);
      speak(say_splited_edge);
    }
  }
  helloDF.step();
  byeDF.step();
  floorDF.step();
  orderReadyDF.step();
  confirmOrderDF.step();
  startDF.step();
  orderDF.step();
  confirmationDF.step();
}
}  // namespace graph_dialog_extractor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grap_dialog_extractor_node");
  ros::NodeHandle n;
  graph_dialog_extractor::GraphDialogExtractor gExtractorNode;
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    gExtractorNode.step();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
