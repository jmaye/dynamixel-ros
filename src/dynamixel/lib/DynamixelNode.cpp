/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "DynamixelNode.h"

#include <chrono>
#include <thread>
#include <ratio>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include <boost/make_shared.hpp>

#include <ros/rate.h>

#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/JointState.h>

#include <libdynamixel/sensor/Controller.h>
#include <libdynamixel/com/SerialPort.h>
#include <libdynamixel/exceptions/IOException.h>
#include <libdynamixel/exceptions/SystemException.h>
#include <libdynamixel/exceptions/BadArgumentException.h>

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  DynamixelNode::DynamixelNode(const ros::NodeHandle& nh) :
      nodeHandle_(nh) {
    // retrieve configurable parameters
    getParameters();

    // init diagnostic engine
    diagnosticUpdater_.setHardwareID(deviceName_);
    diagnosticUpdater_.add("Serial connection", this,
      &DynamixelNode::diagnoseSerialConnection);
    diagnosticUpdater_.add("Motor", this,
      &DynamixelNode::diagnoseMotor);
    jointStatePublisherMinFrequency_ = acquisitionLoopRate_ -
      jointStatePublisherFreqTolPercentage_ * acquisitionLoopRate_;
    jointStatePublisherMaxFrequency_ = acquisitionLoopRate_ +
      jointStatePublisherFreqTolPercentage_ * acquisitionLoopRate_;
    jointStatePublisherFrequencyDiagnostic_ =
      std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
      jointStatePublisherTopic_, diagnosticUpdater_,
      diagnostic_updater::FrequencyStatusParam(
      &jointStatePublisherMinFrequency_, &jointStatePublisherMaxFrequency_, 0.1,
      10));

    // init joint state publisher
    jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>(
      jointStatePublisherTopic_, jointStatePublisherQueueSize_);
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void DynamixelNode::calibrateTime() {
    std::vector<int64_t> samples;
    samples.reserve(calibrateTimeNumPackets_);
    for (size_t i = 0; i < static_cast<size_t>(calibrateTimeNumPackets_); ++i) {
      auto start = std::chrono::steady_clock::now();
      controller_->getPresentPositionAngle(motorId_);
      auto end = std::chrono::steady_clock::now();
      samples.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
        end - start).count());
    }
    std::nth_element(samples.begin(), samples.begin() + samples.size() / 2,
      samples.end());
    timeOffset_ = std::round(samples[samples.size() / 2] / 2.0);
    calibrateTime_ = false;
  }

  void DynamixelNode::diagnoseSerialConnection(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("Serial port device", serialPortDeviceName_);
    status.add("Serial port baud rate", serialPortBaudRate_);
    if (controller_ && controller_->getSerialPort() &&
        controller_->getSerialPort()->isOpen())
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Serial connection opened on %s.",
        controller_->getSerialPort()->getDevice().c_str());
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Serial connection closed on %s.", serialPortDeviceName_.c_str());
  }

  void DynamixelNode::diagnoseMotor(diagnostic_updater::DiagnosticStatusWrapper&
      status) {
    status.add("Time offset [ns]", timeOffset_);
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "OK");
  }

  void DynamixelNode::spin() {
    controller_ = std::make_shared<Controller>(
      std::make_shared<SerialPort>(serialPortDeviceName_, serialPortBaudRate_));
    ros::Rate loopRate(acquisitionLoopRate_);
    while (nodeHandle_.ok()) {
      try {
        if (calibrateTime_)
          calibrateTime();
        auto position = controller_->getPresentPositionAngle(motorId_);
        auto timestamp = ros::Time::now();
        timestamp -= ros::Duration(0, timeOffset_);
        jointStatePublisherFrequencyDiagnostic_->tick();
        if (jointStatePublisher_.getNumSubscribers() > 0) {
          auto jointState = boost::make_shared<sensor_msgs::JointState>();
          jointState->header.stamp = timestamp;
          jointState->header.frame_id = jointStatePublisherFrameId_;
          jointState->name.push_back(deviceName_ + "_joint");
          jointState->position.push_back(position);
          jointStatePublisher_.publish(jointState);
        }
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in " << retryTimeout_
          << " [s]");
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "SystemException: "
          << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in "
          << retryTimeout_ << " [s]");
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      catch (const BadArgumentException<size_t>& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
          << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in " << retryTimeout_
          << " [s]");
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      diagnosticUpdater_.update();
      ros::spinOnce();
      loopRate.sleep();
    }
  }

  void DynamixelNode::getParameters() {
    // miscellaneous parameters
    nodeHandle_.param<double>("ros/acquisition_loop_rate", acquisitionLoopRate_,
      100.0);
    nodeHandle_.param<double>("ros/retry_timeout", retryTimeout_, 1.0);

    // sensor parameters
    nodeHandle_.param<std::string>("sensor/serial_port_device_name",
      serialPortDeviceName_, "/dev/ttyUSB0");
    nodeHandle_.param<int>("sensor/serial_port_baud_rate",
      serialPortBaudRate_, 1000000);
    nodeHandle_.param<std::string>("sensor/device_name", deviceName_,
      "Dynamixel controller");
    nodeHandle_.param<int>("sensor/motor_id", motorId_, 1);
    nodeHandle_.param<bool>("sensor/calibrate_time", calibrateTime_, true);
    nodeHandle_.param<int>("sensor/calibrate_time_num_packets",
      calibrateTimeNumPackets_, 100);

    // joint state publisher parameters
    nodeHandle_.param<std::string>("joint_state_publisher/topic",
      jointStatePublisherTopic_, "joint_state");
    nodeHandle_.param<double>("joint_state_publisher/freq_tol_percentage",
      jointStatePublisherFreqTolPercentage_, 0.1);
    nodeHandle_.param<std::string>("joint_state_publisher/frame_id",
      jointStatePublisherFrameId_, "dynamixel");
  }

}
