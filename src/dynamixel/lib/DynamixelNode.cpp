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

    // init services
    setPidGainsService_ = nodeHandle_.advertiseService("set_pid_gains",
      &DynamixelNode::setPidGains, this);
    getPidGainsService_ = nodeHandle_.advertiseService("get_pid_gains",
      &DynamixelNode::getPidGains, this);
    setComplianceService_ = nodeHandle_.advertiseService("set_compliance",
      &DynamixelNode::setCompliance, this);
    getComplianceService_ = nodeHandle_.advertiseService("get_compliance",
      &DynamixelNode::getCompliance, this);
    setAngleLimitsService_ = nodeHandle_.advertiseService("set_angle_limits",
      &DynamixelNode::setAngleLimits, this);
    getAngleLimitsService_ = nodeHandle_.advertiseService("get_angle_limits",
      &DynamixelNode::getAngleLimits, this);
    setMaxTorqueService_ = nodeHandle_.advertiseService("set_max_torque",
      &DynamixelNode::setMaxTorque, this);
    getMaxTorqueService_ = nodeHandle_.advertiseService("get_max_torque",
      &DynamixelNode::getMaxTorque, this);
    setTorqueEnableService_ = nodeHandle_.advertiseService("set_torque_enable",
      &DynamixelNode::setTorqueEnable, this);
    getTorqueEnableService_ = nodeHandle_.advertiseService("get_torque_enable",
      &DynamixelNode::getTorqueEnable, this);
    setTorqueControlModeEnableService_ = nodeHandle_.advertiseService(
      "set_torque_control_mode_enable",
      &DynamixelNode::setTorqueControlModeEnable, this);
    getTorqueControlModeEnableService_ = nodeHandle_.advertiseService(
      "get_torque_control_mode_enable",
      &DynamixelNode::getTorqueControlModeEnable, this);
    setGoalTorqueService_ = nodeHandle_.advertiseService("set_goal_torque",
      &DynamixelNode::setGoalTorque, this);
    getGoalTorqueService_ = nodeHandle_.advertiseService("get_goal_torque",
      &DynamixelNode::getGoalTorque, this);
    setGoalPositionService_ = nodeHandle_.advertiseService("set_goal_position",
      &DynamixelNode::setGoalPosition, this);
    getGoalPositionService_ = nodeHandle_.advertiseService("get_goal_position",
      &DynamixelNode::getGoalPosition, this);
    setMovingSpeedService_ = nodeHandle_.advertiseService("set_moving_speed",
      &DynamixelNode::setMovingSpeed, this);
    getMovingSpeedService_ = nodeHandle_.advertiseService("get_moving_speed",
      &DynamixelNode::getMovingSpeed, this);
    setGoalAccelerationService_ = nodeHandle_.advertiseService(
      "set_goal_acceleration", &DynamixelNode::setGoalAcceleration, this);
    getGoalAccelerationService_ = nodeHandle_.advertiseService(
      "get_goal_acceleration", &DynamixelNode::getGoalAcceleration, this);
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
      status.add("Return delay time [us]",
        Controller::raw2us(returnDelayTime_));
      status.add("Time offset [ns]", timeOffset_);
      status.add("Clockwise angle limit [rad]", Controller::tick2angle(
        cwAngleLimit_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Counterclockwise angle limit [rad]", Controller::tick2angle(
        ccwAngleLimit_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Highest limit temperature [C]", static_cast<unsigned>(
        highestLimitTemperature_));
      status.add("Highest limit voltage [V]", Controller::raw2volt(
        highestLimitVoltage_));
      status.add("Lowest limit voltage [V]",Controller::raw2volt(
        lowestLimitVoltage_));
      status.add("Maximum torque [%]", Controller::raw2torqueRatio(maxTorque_) *
        100.0);
      status.add("Torque enabled", torqueEnabled_);
      if (Controller::isModelMX(modelNumber_)) {
        status.add("P gain", Controller::raw2Kp(pGain_));
        status.add("I gain", Controller::raw2Ki(iGain_));
        status.add("D gain", Controller::raw2Kd(dGain_));
      }
      else {
        status.add("Clockwise compliance margin", cwComplianceMargin_);
        status.add("Counterclockwise compliance margin", ccwComplianceMargin_);
        status.add("Clockwise compliance slope", cwComplianceSlope_);
        status.add("Counterclockwise compliance slope", ccwComplianceSlope_);
      }
      status.add("Goal position [rad]", Controller::tick2angle(
        goalPosition_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Moving speed [rad/s]", Controller::rpm2rps(
        Controller::raw2rpm(movingSpeed_, rpmPerTick_)));
      status.add("Torque limit [%]", Controller::raw2torqueRatio(torqueLimit_) *
        100.0);
      status.add("Present position [rad]", Controller::tick2angle(
        presentPosition_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
      status.add("Present speed [rad/s]", Controller::rpm2rps(
        Controller::raw2rpm(presentSpeed_, rpmPerTick_)));
      status.add("Present load [%]", Controller::raw2torqueRatio(presentLoad_) *
        100.0);
      status.add("Present voltage [V]", Controller::raw2volt(presentVoltage_));
      status.add("Present temperature [C]", static_cast<unsigned>(
        presentTemperature_));
      status.add("Moving", moving_);
      status.add("Punch", punch_);
      if (Controller::isModelTorqueControl(modelNumber_)) {
        status.add("Current [A]", Controller::raw2amp(current_));
        status.add("Torque control mode enabled", torqueControlModeEnabled_);
        status.add("Goal torque [A]", Controller::rawtorque2amp(goalTorque_));
      }
      if (Controller::isModelMX(modelNumber_))
        status.add("Goal acceleration [rad/s^2]",
          Controller::raw2rps2(goalAcceleration_));
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
          returnDelayTime_ = controller_->getReturnDelayTime(motorId_);
          modelName_ = Controller::getModelInformation(modelNumber_).name;
          maxTicks_ = Controller::getModelInformation(modelNumber_).maxTicks;
          rangeInDegrees_ = Controller::getModelInformation(modelNumber_).
            rangeInDegrees;
          rpmPerTick_ = Controller::getModelInformation(modelNumber_).
            rpmPerTick;
          servoBaudRate_ = controller_->getBaudRate(motorId_);
          controller_->getAngleLimits(motorId_, cwAngleLimit_, ccwAngleLimit_);
          highestLimitTemperature_ = controller_->getHighestLimitTemperature(
            motorId_);
          highestLimitVoltage_ = controller_->getHighestLimitVoltage(motorId_);
          lowestLimitVoltage_ = controller_->getLowestLimitVoltage(motorId_);
          maxTorque_ = controller_->getMaxTorque(motorId_);
          torqueEnabled_ = controller_->isTorqueEnable(motorId_);
          punch_ = controller_->getPunch(motorId_);
          if (Controller::isModelMX(modelNumber_)) {
            controller_->getPIDGains(motorId_, pGain_, iGain_, dGain_);
            goalAcceleration_ = controller_->getGoalAcceleration(motorId_);
          }
          else
            controller_->getCompliance(motorId_, cwComplianceMargin_,
              ccwComplianceMargin_, cwComplianceSlope_, ccwComplianceSlope_);
          controller_->getGoalPositionSpeedTorque(motorId_, goalPosition_,
            movingSpeed_, torqueLimit_);
          if (Controller::isModelTorqueControl(modelNumber_)) {
            current_ = controller_->getCurrent(motorId_);
            torqueControlModeEnabled_ =
              controller_->isTorqueControlModeEnable(motorId_);
            goalTorque_ = controller_->getGoalTorque(motorId_);
          }
          controller_->setReturnDelayTime(motorId_, 0);
        }
        auto start = std::chrono::steady_clock::now();
        controller_->getState(motorId_, presentPosition_, presentSpeed_,
          presentLoad_, presentVoltage_, presentTemperature_, registered_,
          moving_);
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
          jointState->position.push_back(Controller::tick2angle(
            presentPosition_, Controller::deg2rad(rangeInDegrees_), maxTicks_));
          jointStatePublisher_.publish(jointState);
        }
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, Controller::tick2angle(presentPosition_,
          Controller::deg2rad(rangeInDegrees_), maxTicks_));
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

  bool DynamixelNode::setPidGains(dynamixel::SetPidGains::Request& request,
      dynamixel::SetPidGains::Response& response) {
    const auto oldPGain = pGain_;
    const auto oldIGain = iGain_;
    const auto oldDGain = dGain_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setPidGains: motor not connected");
      if (!Controller::isModelMX(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setPidGains: not applicable");
      pGain_ = Controller::Kp2raw(request.p_gain);
      iGain_ = Controller::Ki2raw(request.i_gain);
      dGain_ = Controller::Kd2raw(request.d_gain);
      controller_->setPIDGains(motorId_, pGain_, iGain_, dGain_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "DynamixelNode::setPidGains: "
        "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      pGain_ = oldPGain;
      iGain_ = oldIGain;
      dGain_ = oldDGain;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      pGain_ = oldPGain;
      iGain_ = oldIGain;
      dGain_ = oldDGain;
    }
    return true;
  }

  bool DynamixelNode::getPidGains(dynamixel::GetPidGains::Request& /*request*/,
      dynamixel::GetPidGains::Response& response) {
    if (motorConnected_ && Controller::isModelMX(modelNumber_)) {
      response.p_gain = Controller::raw2Kp(pGain_);
      response.i_gain = Controller::raw2Ki(iGain_);
      response.d_gain = Controller::raw2Kd(dGain_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setCompliance(dynamixel::SetCompliance::Request& request,
      dynamixel::SetCompliance::Response& response) {
    const auto oldCwComplianceMargin = cwComplianceMargin_;
    const auto oldCcwComplianceMargin = ccwComplianceMargin_;
    const auto oldCwComplianceSlope = cwComplianceSlope_;
    const auto oldCcwComplianceSlope = ccwComplianceSlope_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setCompliance: motor not connected");
      if (Controller::isModelMX(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setCompliance: not applicable");
      cwComplianceMargin_ = request.cw_margin;
      ccwComplianceMargin_ = request.ccw_margin;
      cwComplianceSlope_ = request.cw_slope;
      ccwComplianceSlope_ = request.ccw_slope;
      controller_->setCompliance(motorId_, cwComplianceMargin_,
        ccwComplianceMargin_, cwComplianceSlope_, ccwComplianceSlope_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      cwComplianceMargin_ = oldCwComplianceMargin;
      ccwComplianceMargin_ = oldCcwComplianceMargin;
      cwComplianceSlope_ = oldCwComplianceSlope;
      ccwComplianceSlope_ = oldCcwComplianceSlope;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      cwComplianceMargin_ = oldCwComplianceMargin;
      ccwComplianceMargin_ = oldCcwComplianceMargin;
      cwComplianceSlope_ = oldCwComplianceSlope;
      ccwComplianceSlope_ = oldCcwComplianceSlope;
    }
    return true;
  }

  bool DynamixelNode::getCompliance(dynamixel::GetCompliance::Request&
      /*request*/, dynamixel::GetCompliance::Response& response) {
    if (motorConnected_ && !Controller::isModelMX(modelNumber_)) {
      response.cw_margin = cwComplianceMargin_;
      response.ccw_margin = ccwComplianceMargin_;
      response.cw_slope = cwComplianceSlope_;
      response.ccw_slope = ccwComplianceSlope_;
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setAngleLimits(dynamixel::SetAngleLimits::Request&
      request, dynamixel::SetAngleLimits::Response& response) {
    const auto oldCwAngleLimit = cwAngleLimit_;
    const auto oldCcwAngleLimit = ccwAngleLimit_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setAngleLimits: motor not connected");
      cwAngleLimit_ = Controller::angle2tick(request.cw_angle_limit,
        Controller::deg2rad(rangeInDegrees_), maxTicks_);
      ccwAngleLimit_ = Controller::angle2tick(request.ccw_angle_limit,
        Controller::deg2rad(rangeInDegrees_), maxTicks_);
      controller_->setAngleLimits(motorId_, cwAngleLimit_, ccwAngleLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      cwAngleLimit_ = oldCwAngleLimit;
      ccwAngleLimit_ = oldCcwAngleLimit;
    }
    return true;
  }

  bool DynamixelNode::getAngleLimits(dynamixel::GetAngleLimits::Request&
      /*request*/, dynamixel::GetAngleLimits::Response& response) {
    if (motorConnected_) {
      response.cw_angle_limit = Controller::tick2angle(cwAngleLimit_,
        Controller::deg2rad(rangeInDegrees_), maxTicks_);
      response.ccw_angle_limit = Controller::tick2angle(ccwAngleLimit_,
        Controller::deg2rad(rangeInDegrees_), maxTicks_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setMaxTorque(dynamixel::SetMaxTorque::Request& request,
      dynamixel::SetMaxTorque::Response& response) {
    const auto oldMaxTorque = maxTorque_;
    try {
      if (!motorConnected_)
        throw IOException("DynamixelNode::setMaxTorque: motor not connected");
      maxTorque_ = Controller::torqueRatio2raw(request.max_torque);
      controller_->setMaxTorque(motorId_, maxTorque_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      maxTorque_ = oldMaxTorque;
    }
    return true;
  }

  bool DynamixelNode::getMaxTorque(dynamixel::GetMaxTorque::Request&
      /*request*/, dynamixel::GetMaxTorque::Response& response) {
    if (motorConnected_) {
      response.max_torque = Controller::raw2torqueRatio(maxTorque_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setTorqueEnable(dynamixel::SetTorqueEnable::Request&
      request, dynamixel::SetTorqueEnable::Response& response) {
    const auto oldTorqueEnabled = torqueEnabled_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setTorqueEnable: motor not connected");
      torqueEnabled_ = request.enable_torque;
      controller_->setTorqueEnable(motorId_, torqueEnabled_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      torqueEnabled_ = oldTorqueEnabled;
    }
    return true;
  }

  bool DynamixelNode::getTorqueEnable(dynamixel::GetTorqueEnable::Request&
      /*request*/, dynamixel::GetTorqueEnable::Response& response) {
    if (motorConnected_) {
      response.torque_enabled = torqueEnabled_;
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setTorqueControlModeEnable(
      dynamixel::SetTorqueControlModeEnable::Request& request,
      dynamixel::SetTorqueControlModeEnable::Response& response) {
    const auto oldTorqueControlModeEnabled = torqueControlModeEnabled_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setTorqueControlModeEnable: motor not connected");
      if (!Controller::isModelTorqueControl(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setTorqueControlModeEnable: not applicable");
      torqueControlModeEnabled_ = request.enable_torque_control_mode;
      controller_->setTorqueControlModeEnable(motorId_,
        torqueControlModeEnabled_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      torqueControlModeEnabled_ = oldTorqueControlModeEnabled;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      torqueControlModeEnabled_ = oldTorqueControlModeEnabled;
    }
    return true;
  }

  bool DynamixelNode::getTorqueControlModeEnable(
      dynamixel::GetTorqueControlModeEnable::Request& /*request*/,
      dynamixel::GetTorqueControlModeEnable::Response& response) {
    if (motorConnected_ & Controller::isModelTorqueControl(modelNumber_)) {
      response.torque_control_mode_enabled = torqueControlModeEnabled_;
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setGoalTorque(dynamixel::SetGoalTorque::Request& request,
      dynamixel::SetGoalTorque::Response& response) {
    const auto oldGoalTorque = goalTorque_;
    const auto oldTorqueLimit = torqueLimit_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setGoalTorque: motor not connected");
      if (!Controller::isModelTorqueControl(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setGoalTorque: not applicable");
      goalTorque_ = Controller::amp2rawtorque(request.goal_torque);
      torqueLimit_ = Controller::torqueRatio2raw(request.torque_limit);
      if (goalTorque_ != oldGoalTorque)
        controller_->setGoalTorque(motorId_, goalTorque_);
      if (torqueLimit_ != oldTorqueLimit)
        controller_->setTorqueLimit(motorId_, torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      goalTorque_ = oldGoalTorque;
      torqueLimit_ = oldTorqueLimit;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      goalTorque_ = oldGoalTorque;
      torqueLimit_ = oldTorqueLimit;
    }
    return true;
  }

  bool DynamixelNode::getGoalTorque(dynamixel::GetGoalTorque::Request&
      /*request*/, dynamixel::GetGoalTorque::Response& response) {
    if (motorConnected_ & Controller::isModelTorqueControl(modelNumber_)) {
      response.goal_torque = Controller::rawtorque2amp(goalTorque_);
      response.torque_limit = Controller::raw2torqueRatio(torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

  bool DynamixelNode::setGoalPosition(dynamixel::SetGoalPosition::Request&
      request, dynamixel::SetGoalPosition::Response& response) {
    const auto oldGoalPosition = goalPosition_;
    const auto oldMovingSpeed = movingSpeed_;
    const auto oldTorqueLimit = torqueLimit_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setGoalPosition: motor not connected");
      goalPosition_ = Controller::angle2tick(request.goal_position,
        Controller::deg2rad(rangeInDegrees_), maxTicks_);
      movingSpeed_ = Controller::rpm2raw(Controller::rps2rpm(
        request.moving_speed), rpmPerTick_);
      torqueLimit_ = Controller::torqueRatio2raw(request.torque_limit);
      if (goalPosition_ != oldGoalPosition && movingSpeed_ == oldMovingSpeed
          && torqueLimit_ == oldTorqueLimit)
        controller_->setGoalPosition(motorId_, goalPosition_);
      else if (goalPosition_ != oldGoalPosition
          && movingSpeed_ != oldMovingSpeed && torqueLimit_ == oldTorqueLimit)
        controller_->setGoalPositionSpeed(motorId_, goalPosition_,
          movingSpeed_);
      else if (goalPosition_ != oldGoalPosition
          && movingSpeed_ != oldMovingSpeed && torqueLimit_ != oldTorqueLimit)
        controller_->setGoalPositionSpeedTorque(motorId_, goalPosition_,
          movingSpeed_, torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      goalPosition_ = oldGoalPosition;
      movingSpeed_ = oldMovingSpeed;
      torqueLimit_ = oldTorqueLimit;
    }
    return true;
  }

  bool DynamixelNode::getGoalPosition(dynamixel::GetGoalPosition::Request&
      /*request*/, dynamixel::GetGoalPosition::Response& response) {
    if (motorConnected_) {
      response.goal_position = Controller::tick2angle(goalPosition_,
        Controller::deg2rad(rangeInDegrees_), maxTicks_);
      response.moving_speed = Controller::rpm2rps(
        Controller::raw2rpm(movingSpeed_, rpmPerTick_));
      response.torque_limit = Controller::raw2torqueRatio(torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setMovingSpeed(dynamixel::SetMovingSpeed::Request&
      request, dynamixel::SetMovingSpeed::Response& response) {
    const auto oldMovingSpeed = movingSpeed_;
    const auto oldTorqueLimit = torqueLimit_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setMovingSpeed: motor not connected");
      movingSpeed_ = Controller::rpm2raw(Controller::rps2rpm(
        request.moving_speed), rpmPerTick_);
      torqueLimit_ = Controller::torqueRatio2raw(request.torque_limit);
      if (movingSpeed_ != oldMovingSpeed)
        controller_->setMovingSpeed(motorId_, movingSpeed_);
      if (torqueLimit_ != oldTorqueLimit)
        controller_->setTorqueLimit(motorId_, torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      movingSpeed_ = oldMovingSpeed;
      torqueLimit_ = oldTorqueLimit;
    }
    return true;
  }

  bool DynamixelNode::getMovingSpeed(dynamixel::GetMovingSpeed::Request&
      /*request*/, dynamixel::GetMovingSpeed::Response& response) {
    if (motorConnected_) {
      response.moving_speed = Controller::rpm2rps(
        Controller::raw2rpm(movingSpeed_, rpmPerTick_));
      response.torque_limit = Controller::raw2torqueRatio(torqueLimit_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected";
    }
    return true;
  }

  bool DynamixelNode::setGoalAcceleration(
      dynamixel::SetGoalAcceleration::Request& request,
      dynamixel::SetGoalAcceleration::Response& response) {
    const auto oldGoalAcceleration = goalAcceleration_;
    try {
      if (!motorConnected_)
        throw IOException(
          "DynamixelNode::setGoalAcceleration: motor not connected");
      if (!Controller::isModelMX(modelNumber_))
        throw BadArgumentException<size_t>(modelNumber_,
          "DynamixelNode::setGoalAcceleration: not applicable");
      goalAcceleration_ = Controller::rps22raw(request.goal_acceleration);
      if (goalAcceleration_ != oldGoalAcceleration)
        controller_->setGoalAcceleration(motorId_, goalAcceleration_);
      response.response = true;
      response.message = "Success";
    }
    catch (const IOException& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "IOException: " << e.what());
      response.response = false;
      response.message = e.what();
      goalAcceleration_ = oldGoalAcceleration;
    }
    catch (const BadArgumentException<size_t>& e) {
      ROS_WARN_STREAM_NAMED("dynamixel_node", "BadArgumentException: "
        << e.what());
      response.response = false;
      response.message = e.what();
      goalAcceleration_ = oldGoalAcceleration;
    }
    return true;
  }

  bool DynamixelNode::getGoalAcceleration(
      dynamixel::GetGoalAcceleration::Request& /*request*/,
      dynamixel::GetGoalAcceleration::Response& response) {
    if (motorConnected_ & Controller::isModelMX(modelNumber_)) {
      response.goal_acceleration = Controller::raw2rps2(goalAcceleration_);
      response.response = true;
      response.message = "Success";
    }
    else {
      response.response = false;
      response.message = "Motor not connected or not applicable";
    }
    return true;
  }

}
