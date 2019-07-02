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
#include <gb_dialog/DialogInterface.h>

#include <string>

namespace gb_dialog
{
DialogInterface::DialogInterface(std::string intent) :
  intent_(intent), nh_()
{
  init();
}

DialogInterface::DialogInterface() : nh_()
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
  talk_client_ = nh_.serviceClient<sound_play::Talk>("/gb_dialog/talk");

  idle_ = true;
  setCallTime(ros::Time::now());
}

std::string DialogInterface::getIntent()
{
  return intent_;
}

std::regex DialogInterface::getIntentRegex()
{
  std::regex intent_re_(intent_);
  return intent_re_;
}

void DialogInterface::dfCallback(const dialogflow_ros_msgs::DialogflowResult::ConstPtr& result)
{
  std::regex intent_re_(intent_);
  if (std::regex_match(result->intent, intent_re_) && result->intent.size() > 0)
  {
    setCallTime(ros::Time::now());
    listenCallback(*result);
  }
}

bool DialogInterface::speak(std::string str)
{
  sound_play::Talk srv;
  srv.request.str = str;
  talk_client_.call(srv);
}

bool DialogInterface::listen()
{
  ROS_INFO("[DialogInterface] listening...");
  std_srvs::Empty srv;
  ros::ServiceClient df_srv = nh_.serviceClient<std_srvs::Empty>(start_srv_, 1);
  df_srv.call(srv);
  return true;
}

void DialogInterface::setIdleState(bool state)
{
  idle_ = state;
}

bool DialogInterface::isIdle()
{
  return idle_;
}

void DialogInterface::setCallTime(ros::Time t)
{
  last_call_ = t;
}

ros::Time DialogInterface::getCallTime()
{
  return last_call_;
}

};  // namespace gb_dialog
