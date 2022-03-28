/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics Lab
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

/* Author: Jonatan Gines jonatan.gines@urjc.es */

/* Mantainer: Jonatan Gines jonatan.gines@urjc.es */
#include <gb_dialog/DialogInterface.h>

#include <string>
#include <utility>

namespace gb_dialog
{

DialogInterface::DialogInterface()
: nh_(),
  sc_(nh_, "/robotsound")
{
  init();
}

void DialogInterface::init()
{
  if (!nh_.getParam("/dialogflow_client/results_topic", results_topic_))
    results_topic_ = "/dialogflow_client/results";
  if (!nh_.getParam("/dialogflow_client/start_srv", start_srv_))
    start_srv_ = "/dialogflow_client/start";

  df_result_sub_ = nh_.subscribe(results_topic_, 1, &DialogInterface::dfCallback, this);
  listening_gui_ = nh_.advertise<std_msgs::Bool>("/dialog_gui/is_listening", 1, true);
  speak_gui_ = nh_.advertise<std_msgs::String>("/dialog_gui/talk", 1, true);

  std_msgs::String str_msg;
  std_msgs::Bool bool_msg;
  str_msg.data = "";
  speak_gui_.publish(str_msg);
  bool_msg.data = false;
  listening_gui_.publish(bool_msg);
}

void DialogInterface::registerCallback(
  std::function<void(const DialogflowResult & result)> cb,
  std::string intent)
{
  registered_cbs_.insert(
    std::pair<std::string, std::function<void(const DialogflowResult & result)>>
    (intent, cb));
}

void DialogInterface::dfCallback(const DialogflowResult::ConstPtr& result)
{ 
  auto bool_msg = std_msgs::Bool();
  bool_msg.data = false;
  listening_gui_.publish(bool_msg);

  if (result->intent.size() > 0)
  {
    for (auto item : registered_cbs_)
    {
      std::regex intent_re = std::regex(item.first);
      if (std::regex_match(result->intent, intent_re))
      {
        
        item.second(*result);
      }
    }
  }
}

bool DialogInterface::speak(std::string str)
{
  boost::replace_all(str, "_", " ");
  std_msgs::String msg;
  msg.data = str;
  speak_gui_.publish(msg);

  sc_.say(str);
}

bool DialogInterface::listen()
{
  std_srvs::Empty srv;
  std_msgs::Bool msg;
  msg.data = true;
  listening_gui_.publish(msg);
  ROS_INFO("[DialogInterface] listening...");
  ros::ServiceClient df_srv = nh_.serviceClient<std_srvs::Empty>(start_srv_, 1);
  df_srv.call(srv);
  return true;
}

};  // namespace gb_dialog
