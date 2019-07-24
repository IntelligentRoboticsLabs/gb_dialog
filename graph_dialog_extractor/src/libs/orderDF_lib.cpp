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
#include <graph_dialog_extractor/orderDF_lib.h>
#include <string>
#include <algorithm>
#include <cctype>
#include <list>

namespace graph_dialog_extractor
{
OrderDF::OrderDF(std::string intent):
  DialogInterface(intent), nh_("~"), edge_()
{
  intent_ = intent;
}

void OrderDF::listenCallback(dialogflow_ros_msgs::DialogflowResult result)
{
  if (edge_ == NULL)
    return;
  ROS_INFO("[OrderDF] listenCallback: intent %s", result.intent.c_str());
  std::string total_items;
  if (result.fulfillment_text != "")
  {
    ROS_WARN("[OrderDF] %s", result.fulfillment_text.c_str());
    speak(result.fulfillment_text);
  }
  else
  {
    graph_.remove_edge(*edge_);
    for (int i = 0; i < result.parameters.size(); i++)
    {
      for (int j = 0; j  < result.parameters[i].value.size(); j++)
      {
        ROS_INFO("[GraphDialogExtractor] listenCallback: parameter %s value %s",
          result.parameters[i].param_name.c_str(),
          result.parameters[i].value[j].c_str());

        std::string item = result.parameters[i].value[j];
        std::transform(item.begin(), item.end(), item.begin(),
            [](unsigned char c){ return std::tolower(c); });
        if (i == 0)
          total_items = item;
        else
          total_items = total_items + " " + item;
      }
    }

    speak("You have ordered " + total_items);
    graph_.add_edge(edge_->get_target(), "response: " + total_items, edge_->get_source());
    edge_ = NULL;
  }
}

void OrderDF::step()
{
  std::list<bica_graph::StringEdge> edges_list =  graph_.get_string_edges();
  for (auto it = edges_list.begin(); it != edges_list.end(); ++it)
  {
    std::string edge = it->get();
    if (edge.find("ask: " + intent_) != std::string::npos)
    {
      speak("Hello, What are you going to take?");
      edge_ = new bica_graph::StringEdge(*it);
      ROS_INFO("[Ask] %s", edge.c_str());
      listen();
    }
  }
}

};  // namespace graph_dialog_extractor
