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

/* Author: Jonatan Ginés jginesclavero@gmail.com */

/* Mantainer: Jonatan Gines jginesclavero@gmail.com */

#include "gb_dialog/DialogInterface.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/package.h>
#include <regex>

namespace gb_dialog {
class DialogInterfaceTest: public gb_dialog::DialogInterface
{
  public:
    std::string intent_;
    explicit DialogInterfaceTest(std::string intent): DialogInterface(intent){}
    void listenCallback(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[DialogInterfaceTest] listenCallback: intent %s", result.intent.c_str());
      intent_ = result.intent;
    }
};

class DialogInterfaceTestRe: public gb_dialog::DialogInterface
{
  public:
    dialogflow_ros_msgs::DialogflowResult result_;
    DialogInterfaceTestRe(std::regex intent): DialogInterface(intent){}
    void listenCallback(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[DialogInterfaceTest] listenCallback: intent %s", result.intent.c_str());
      result_ = result;
    }
};
}  // namespace gb_dialog

TEST(TESTSuite, instance_lib)
{
  std::string intent_in = "Default Welcome Intent", intent_out;
  gb_dialog::DialogInterfaceTest di(intent_in);
  intent_out = di.getIntent();
  if (intent_out != "")
    EXPECT_EQ(intent_out, intent_in);
}

TEST(TESTSuite, speak)
{
  std::string intent_in = "Default Welcome Intent";
  gb_dialog::DialogInterfaceTest di(intent_in);
  EXPECT_TRUE(di.speak("Hello world!"));
}

TEST(TESTSuite, instance_lib_re)
{
  std::string intent_out;
  std::regex intent_in("[[:print:]_]*.info");
  gb_dialog::DialogInterfaceTestRe di(intent_in);

  di.listen();
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.3) > ros::Time::now()) ros::spinOnce();

  EXPECT_TRUE(di.speak(di.result_.fulfillment_text));
}

TEST(TESTSuite, listen)
{
  std::string intent_in = "Default Welcome Intent";
  gb_dialog::DialogInterfaceTest di(intent_in);
  di.listen();

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.3) > ros::Time::now()) ros::spinOnce();

  EXPECT_EQ(di.intent_ , intent_in);
}

//TEST(TESTSuite, listen_loop)
//{
//  std::string intent_in = "Default Welcome Intent";
//  gb_dialog::DialogInterfaceTest di(intent_in);
//
//  ros::AsyncSpinner spinner(2);
//  spinner.start();
//
//  while (ros::ok())
//  {
//    di.listen();
//  }
//
//  //EXPECT_EQ(di.intent_ , intent_in);
//}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_DialogInterface");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
