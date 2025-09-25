/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** 
 * Simple stand-alone ROS node that takes data from NetFT sensor and
 * Publishes it ROS topic
 */

#include <geometry_msgs/msg/wrench_stamped.h>
#include <netft_utils/netft_rdt_driver.h>

#include <boost/program_options.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Filter out ROS 2-specific arguments
  std::vector<char *> filtered_args;
  for (int i = 0; i < argc; ++i) {
    if (std::string(argv[i]) != "--ros-args") {
      filtered_args.push_back(argv[i]);
    }
  }
  filtered_args.push_back(nullptr); // Add null terminator

  int filtered_argc = static_cast<int>(filtered_args.size()) - 1;
  char ** filtered_argv = filtered_args.data();

  float pub_rate_hz;
  string address;
  string frame_id;

  po::options_description desc("Options");
  desc.add_options()("--ros-args", "ros arguments")("help", "display help")(
    "rate", po::value<float>(&pub_rate_hz)->default_value(500.0), "set publish rate (in hertz)")(
    "address", po::value<string>(&address), "IP address of NetFT box")(
    "frame_id", po::value<string>(&frame_id)->default_value("base_link"),
    "frame_id for Wrench msgs")(
    "topic_name", po::value<string>()->default_value(""), "topic name for wrench output"
    );

  po::positional_options_description p;
  p.add("address", 1);
  p.add("frame_id", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(filtered_argc, filtered_argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << endl;
    //usage(progname);
    exit(EXIT_SUCCESS);
  }

  if (!vm.count("address")) {
    cout << desc << endl;
    cerr << "Please specify address of NetFT" << endl;
    exit(EXIT_FAILURE);
  }

  std_msgs::msg::Bool is_ready;
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> netft;

  // Set up ROS publishers
  auto node = std::make_shared<rclcpp::Node>("netft_node");
  const rclcpp::QoS qos(10);
  string topic_name;
  if (vm.count("topic_name")) {
    topic_name = vm["topic_name"].as<string>();
  } else {
    topic_name = ""; // Default value if topic_name is not provided
  }
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub = node->create_publisher<std_msgs::msg::Bool>("netft_ready" + topic_name, qos);
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr geo_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>("netft_data" + topic_name, 100);

  try {
    netft = std::make_shared<netft_rdt_driver::NetFTRDTDriver>(address, frame_id);
    is_ready.data = true;
    ready_pub->publish(is_ready);
  } catch (std::runtime_error & e) {
    is_ready.data = false;
    ready_pub->publish(is_ready);
    RCLCPP_ERROR_STREAM(node->get_logger(), "Error opening NetFT: " << e.what());
  }

  rclcpp::Rate pub_rate(pub_rate_hz);
  geometry_msgs::msg::WrenchStamped data;

  // rclcpp::Duration diag_pub_duration(std::chrono::milliseconds(1000));
  // rclcpp::Publisher diag_pub = netft->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 2);
  // diagnostic_msgs::DiagnosticArray diag_array;
  // diag_array.status.reserve(1);
  // diagnostic_updater::DiagnosticStatusWrapper diag_status;
  // rclcpp::Time last_diag_pub_time(rclcpp::Clock().now());

  while (rclcpp::ok()) {
    // Publish netft data when data is recieved.
    if (netft->waitForNewData()) {
      netft->getData(data);
      geo_pub->publish(data);
    }

    // TODO: removed diagnostics for now will implement in the future.
    // rclcpp::Time current_time(rclcpp::Clock().now());
    // if ( (current_time - last_diag_pub_time) > diag_pub_duration )
    // {
    //   diag_array.status.clear();
    //   netft->diagnostics(diag_status);
    //   diag_array.status.push_back(diag_status);
    //   diag_array.header.stamp = rclcpp::Clock().now();
    //   diag_pub.publish(diag_array);
    //   ready_pub->publish(is_ready);
    //   last_diag_pub_time = current_time;
    // }

    rclcpp::spin_some(node);
    pub_rate.sleep();
  }

  return 0;
}
