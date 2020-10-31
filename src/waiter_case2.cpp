// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class Waiter : public rclcpp::Node
{
public:
  Waiter() : Node("minimal_subscriber"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    tf2_ros::CreateTimerInterface::SharedPtr cti = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(cti);
    timeout_ms_ = std::chrono::milliseconds{this->declare_parameter("timeout_ms").get<int>()};
    RCLCPP_INFO(get_logger(), "timeout is %dms", timeout_ms_);

    // blockUntilVehiclePositionAvailable();
    blockUntilVehiclePositionAvailable2();
  }

private:

  void blockUntilVehiclePositionAvailable()
  {
    while (!transform_available_ && rclcpp::ok()) {
      static constexpr auto input = "a", output = "b";
      RCLCPP_INFO(
                  get_logger(), "waiting %d ms for %s->%s transform to become available",
                  timeout_ms_.count(), input, output);
      auto callback = [this](const std::shared_future<geometry_msgs::msg::TransformStamped> & future) {
        RCLCPP_INFO(get_logger(), "inside callback ");
        try {
          auto tf = future.get();
          transform_available_ = true;
          RCLCPP_INFO(get_logger(), "transform now available");
          throw 5;
        } catch (const tf2::TimeoutException & e) {
          RCLCPP_INFO(get_logger(), "hit TimeoutException: %s", e.what());
        }
      };
      auto future = tf_buffer_.waitForTransform(
                                  input, output, tf2::TimePointZero, std::chrono::milliseconds(timeout_ms_), callback);
      future.wait_for(timeout_ms_);
      RCLCPP_INFO(get_logger(), "transform available after timeout? %d", transform_available_);
    }
}

  void blockUntilVehiclePositionAvailable2()
  {
    static constexpr auto input = "a", output = "b";
    // tf_buffer_.setUsingDedicatedThread(true);
    std::string msg;
    // first call fails even if transform has been published. Why?
    tf_buffer_.canTransform(input, output, tf2::TimePointZero, 0ms, &msg);
      std::this_thread::sleep_for(3ms);

    // If I don't use 0 ms timeout, warning about frame not existing appear
    while(!tf_buffer_.canTransform(input, output, tf2::TimePointZero, 0ms, &msg) && rclcpp::ok()) {
      RCLCPP_INFO(
                  get_logger(), "waiting %d ms for %s->%s transform to become available",
                  timeout_ms_.count(), input, output);
      std::this_thread::sleep_for(timeout_ms_);
    }
    RCLCPP_INFO(get_logger(), "transform available");
    auto tf = tf_buffer_.lookupTransform(input, output, tf2::TimePointZero);

  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::chrono::milliseconds timeout_ms_;
  bool transform_available_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<Waiter>());
  rclcpp::shutdown();
  return 0;
}
