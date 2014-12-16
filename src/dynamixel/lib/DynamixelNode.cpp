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
      nodeHandle_(nh),
      modelNumber_(0),
      motorConnected_(false) {
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
    if (motorConnected_) {
      status.add("Motor ID", static_cast<unsigned>(motorId_));
      status.add("Baud rate", 2000000.0 / (servoBaudRate_ + 1));
      status.add("Model number", modelNumber_);
      status.add("Model name", modelName_);
      status.add("Firmware version", static_cast<unsigned>(firmwareVersion_));
      status.add("Ticks number", maxTicks_ + 1);
      status.add("Range [deg]", rangeInDegrees_);
      status.add("Rpm per tick", rpmPerTick_);
      status.add("Return delay time [us]", returnDelayTime_);
      status.add("Time offset [ns]", timeOffset_);
      status.add("Clockwise angle limit [tick]", cwAngleLimit_);
      status.add("Counterclockwise angle limit [tick]", ccwAngleLimit_);
      status.add("Highest limit temperature [C]", static_cast<unsigned>(
        highestLimitTemperature_));
      status.add("Highest limit voltage [V]", highestLimitVoltage_);
      status.add("Lowest limit voltage [V]", lowestLimitVoltage_);
      status.add("Maximum torque [%]", maxTorque_);
      status.add("Torque enabled", torqueEnabled_);
      status.add("Current position [rad]", currentPosition_);
      status.add("Goal position [rad]", goalPosition_);
      status.add("Moving speed [rad/s]", movingSpeed_);
      status.add("Torque limit [%]", torqueLimit_);
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor connected");
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "No motor connected");
  }

  void DynamixelNode::spin() {
    controller_ = std::make_shared<Controller>(
      std::make_shared<SerialPort>(serialPortDeviceName_, serialPortBaudRate_));
    ros::Rate loopRate(acquisitionLoopRate_);
    while (nodeHandle_.ok()) {
      try {
        if (!modelNumber_) {
          modelNumber_ = controller_->getModelNumber(motorId_);
          if (!Controller::isModelSupported(modelNumber_))
            throw BadArgumentException<size_t>(modelNumber_,
              "DynamixelNode::spin(): model not supported");
          firmwareVersion_ = controller_->getFirmwareVersion(motorId_);
          returnDelayTime_ = controller_->getReturnDelayTimeUs(motorId_);
          modelName_ = Controller::getModelInformation(modelNumber_).name;
          maxTicks_ = Controller::getModelInformation(modelNumber_).maxTicks;
          rangeInDegrees_ = Controller::getModelInformation(modelNumber_).
            rangeInDegrees;
          rpmPerTick_ = Controller::getModelInformation(modelNumber_).
            rpmPerTick;
          servoBaudRate_ = controller_->getBaudRate(motorId_);
          cwAngleLimit_ = controller_->getCwAngleLimitAngle(motorId_);
          ccwAngleLimit_ = controller_->getCcwAngleLimitAngle(motorId_);
          highestLimitTemperature_ = controller_->getHighestLimitTemperature(
            motorId_);
          highestLimitVoltage_ = controller_->getHighestLimitVoltageVolt(
            motorId_);
          lowestLimitVoltage_ = controller_->getLowestLimitVoltageVolt(
            motorId_);
          maxTorque_ = controller_->getMaxTorquePercent(motorId_);
          torqueEnabled_ = controller_->isTorqueEnable(motorId_);
          goalPosition_ = controller_->getGoalPositionAngle(motorId_,
            Controller::deg2rad(rangeInDegrees_), maxTicks_);
          movingSpeed_ = Controller::revPerMin2RadPerSec(
            controller_->getMovingSpeedRpm(motorId_, rpmPerTick_));
          torqueLimit_ = controller_->getTorqueLimitPercent(motorId_);
        }
        auto start = std::chrono::steady_clock::now();
        currentPosition_ = controller_->getPresentPositionAngle(motorId_,
          Controller::deg2rad(rangeInDegrees_), maxTicks_);
        auto end = std::chrono::steady_clock::now();
        auto timestamp = ros::Time::now();
        timeOffset_ = std::round(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
          end - start).count() / 2.0);
        timestamp -= ros::Duration(0, timeOffset_);
        jointStatePublisherFrequencyDiagnostic_->tick();
        motorConnected_ = true;
        if (jointStatePublisher_.getNumSubscribers() > 0) {
          auto jointState = boost::make_shared<sensor_msgs::JointState>();
          jointState->header.stamp = timestamp;
          jointState->header.frame_id = jointStatePublisherFrameId_;
          jointState->name.push_back(jointStatePublisherFrameId_ + "_joint");
          jointState->position.push_back(currentPosition_);
          jointStatePublisher_.publish(jointState);
        }
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, currentPosition_);
        transform.setRotation(q);
        transformBroadcaster_.sendTransform(tf::StampedTransform(transform,
          timestamp, jointStatePublisherFrameId_, jointStatePublisherFrameId_ +
          "_servo"));
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in " << retryTimeout_
          << " [s]");
        motorConnected_ = false;
        modelNumber_ = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "SystemException: "
          << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in "
          << retryTimeout_ << " [s]");
        motorConnected_ = false;
        modelNumber_ = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int64_t>(std::round(retryTimeout_ * std::milli::den))));
      }
      catch (const BadArgumentException<size_t>& e) {
        ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
          << e.what());
        ROS_WARN_STREAM_NAMED("dynamixel_node", "Retrying in " << retryTimeout_
          << " [s]");
        motorConnected_ = false;
        modelNumber_ = 0;
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

    // joint state publisher parameters
    nodeHandle_.param<std::string>("joint_state_publisher/topic",
      jointStatePublisherTopic_, "joint_state");
    nodeHandle_.param<int>("joint_state_publisher/queue_size",
      jointStatePublisherQueueSize_, 100);
    nodeHandle_.param<std::string>("joint_state_publisher/frame_id",
      jointStatePublisherFrameId_, "dynamixel");
    nodeHandle_.param<double>("joint_state_publisher/freq_tol_percentage",
      jointStatePublisherFreqTolPercentage_, 0.1);
  }

}
