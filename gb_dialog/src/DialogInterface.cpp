/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2023, Intelligent Robotics Lab
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
/* Modified: Juan Carlos Manzanares juancarlos.serrano@urjc.es */

/* Mantainer: Jonatan Gines jonatan.gines@urjc.es */
#include "gb_dialog/DialogInterface.hpp"

#include <string>
#include <utility>

using namespace std::placeholders;  

namespace gb_dialog
{

DialogInterface::DialogInterface()
: Node("dialog_interface"),
  sc_("/robotsound")
{
  init();
}

void DialogInterface::init()
{
  results_topic_ = "/dialogflow_client/results";
  start_srv_ = "/dialogflow_client/start";

  df_result_sub_ = create_subscription<DialogflowResult>(
      "results_topic_", 1,
      std::bind(&DialogInterface::dfCallback, this, _1));

  listening_gui_ = this->create_publisher<std_msgs::msg::Bool>("/dialog_gui/is_listening", 1);
  speak_gui_ = this->create_publisher<std_msgs::msg::String>("/dialog_gui/talk", 1);

  std_msgs::msg::String str_msg;
  std_msgs::msg::Bool bool_msg;
  str_msg.data = "";
  speak_gui_->publish(str_msg);
  bool_msg.data = false;
  listening_gui_->publish(bool_msg);
}

void DialogInterface::registerCallback(
  std::function<void(const DialogflowResult & result)> cb,
  std::string intent)
{
  registered_cbs_.insert(
    std::pair<std::string, std::function<void(const DialogflowResult & result)>>
    (intent, cb));
}

void DialogInterface::dfCallback(const DialogflowResult::SharedPtr result)
{ 
  auto bool_msg = std_msgs::msg::Bool();
  bool_msg.data = false;
  listening_gui_->publish(bool_msg);

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
  std_msgs::msg::String msg;
  msg.data = str;
  speak_gui_->publish(msg);

  sc_.say(str);

  return true;
}

bool DialogInterface::listen()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  listening_gui_->publish(msg);
  RCLCPP_INFO(get_logger(), "[DialogInterface] listening...");

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto df_srv = create_client<std_srvs::srv::Empty>(start_srv_);
  auto future = df_srv->async_send_request(request);
  auto response = future.get();

  RCLCPP_INFO(get_logger(), "[DialogInterface] listening stop...");

  return true;
}

};  // namespace gb_dialog
