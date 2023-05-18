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
#include "sound_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace gb_dialog
{

class ExampleDF: public DialogInterface
{
  public:
    ExampleDF()
    {
      registerCallback(std::bind(&ExampleDF::noIntentCB, this, ph::_1));
      registerCallback(
        std::bind(&ExampleDF::welcomeIntentCB, this, ph::_1),
        "Default Welcome Intent");
      registerCallback(
        std::bind(&ExampleDF::requestNameIntentCB, this, ph::_1),
        "RequestName");
      registerCallback(
        std::bind(&ExampleDF::getFoodIntentCB, this, ph::_1),
        "GetFood");
    }

    void noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(node_->get_logger(), "[ExampleDF] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(node_->get_logger(), "[ExampleDF] welcomeIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void requestNameIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(node_->get_logger(), "[ExampleDF] requestNameIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }

    void getFoodIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
    {
      RCLCPP_INFO(node_->get_logger(), "[ExampleDF] getFoodIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
    }
  private:
};
}  // namespace gb_dialog

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("example_dialog");

  auto forwarder = std::make_shared<gb_dialog::ExampleDF>();
  forwarder->init(node);
  forwarder->listen();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
