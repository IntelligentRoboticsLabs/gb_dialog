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
#ifndef GB_DIALOG__DIALOGINTERFACE__HPP
#define GB_DIALOG__DIALOGINTERFACE__HPP

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <dialogflow_ros2_interfaces/msg/dialogflow_result.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "sound_play.hpp"
#include <regex>
#include <map>

namespace gb_dialog
{
class DialogInterface
{
public:
  using DialogflowResult = dialogflow_ros2_interfaces::msg::DialogflowResult;

  DialogInterface();
  void init(rclcpp::Node::SharedPtr node);

  bool speak(std::string str);
  bool listen();
  virtual void listenCallback(DialogflowResult result){}
  void registerCallback(
    std::function<void(const DialogflowResult & result)> cb,
    std::string intent = "(.*)");

protected:
  rclcpp::Node::SharedPtr node_;

private:
  bool idle_;
  std::string results_topic_, start_srv_;
  rclcpp::Subscription<DialogflowResult>::SharedPtr df_result_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr listening_gui_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speak_gui_;
  std::regex intent_re_;

  std::map<std::string, std::function<void(const DialogflowResult & result)>> registered_cbs_;

  sound_play::SoundClient sc_;

  void dfCallback(const DialogflowResult::SharedPtr result);
};
};  // namespace gb_dialog

#endif  // DIALOGINTERFACE__HPP
