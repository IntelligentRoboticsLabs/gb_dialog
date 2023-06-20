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
/* Mantainer: Juan Carlos Manzanares juancarlos.serrano@urjc.es */
#include "gb_dialog/DialogInterface.hpp"

#include <string>
#include <utility>

using namespace std::placeholders;  

namespace gb_dialog
{

DialogInterface::DialogInterface()
: sc_("/robotsound")
{
}

void DialogInterface::init(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  results_topic_ = "/dialogflow_client/results";
  start_srv_ = "/dialogflow_client/start";

  df_result_sub_ = node_->create_subscription<DialogflowResult>(
      results_topic_, 1,
      std::bind(&DialogInterface::dfCallback, this, _1));

  listening_gui_ = node_->create_publisher<std_msgs::msg::Bool>("/dialog_gui/is_listening", 1);
  speak_gui_ = node_->create_publisher<std_msgs::msg::String>("/dialog_gui/talk", 1);

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

bool DialogInterface::speak(std::string str, const std::string &voice, float volume)
{
  boost::replace_all(str, "_", " ");
  std_msgs::msg::String msg;
  msg.data = str;
  speak_gui_->publish(msg);

  sc_.say(str, voice, volume);

  return true;
}

bool
DialogInterface::listen()
{
  std_msgs::msg::Bool msg;
  msg.data = true;
  listening_gui_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[DialogInterface] listening...");

  auto node_aux = rclcpp::Node::make_shared("example_dialog_aux");

  auto df_srv = node_aux->create_client<std_srvs::srv::Empty>(start_srv_);

  while (!df_srv->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node_->get_logger(), "Service not available, waiting again...");
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = df_srv->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_aux, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "service call failed :(");
    df_srv->remove_pending_request(result);
    return false;
  }

  return true;
}

};  // namespace gb_dialog
