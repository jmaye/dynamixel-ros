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

/** \file DynamixelNode.h
    \brief This file defines the DynamixelNode class which implements the
           Dynamixel node.
  */

#ifndef DYNAMIXEL_NODE_H
#define DYNAMIXEL_NODE_H

#include <string>
#include <memory>
#include <cstdint>

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <tf/transform_broadcaster.h>

#include "dynamixel/SetPidGains.h"
#include "dynamixel/GetPidGains.h"
#include "dynamixel/SetCompliance.h"
#include "dynamixel/GetCompliance.h"
#include "dynamixel/SetAngleLimits.h"
#include "dynamixel/GetAngleLimits.h"
#include "dynamixel/SetMaxTorque.h"
#include "dynamixel/GetMaxTorque.h"
#include "dynamixel/SetTorqueEnable.h"
#include "dynamixel/GetTorqueEnable.h"
#include "dynamixel/SetTorqueControlModeEnable.h"
#include "dynamixel/GetTorqueControlModeEnable.h"

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace dynamixel {

  class Controller;

  /** The class DynamixelNode implements the Dynamixel node.
      \brief Dynamixel node
    */
  class DynamixelNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    DynamixelNode(const ros::NodeHandle& nh);
    /// Copy constructor
    DynamixelNode(const DynamixelNode& other) = delete;
    /// Copy assignment operator
    DynamixelNode& operator = (const DynamixelNode& other) = delete;
    /// Move constructor
    DynamixelNode(DynamixelNode&& other) = delete;
    /// Move assignment operator
    DynamixelNode& operator = (DynamixelNode&& other) = delete;
    /// Destructor
    ~DynamixelNode() = default;
    /** @}
      */

    /** \name Public Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
    /// Retrieves parameters from the parameter server
    void getParameters();
    /// Diagnose the serial connection
    void diagnoseSerialConnection(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Diagnose the servo motor
    void diagnoseMotor(diagnostic_updater::DiagnosticStatusWrapper& status);
    /// Set PID gains service
    bool setPidGains(dynamixel::SetPidGains::Request& request,
      dynamixel::SetPidGains::Response& response);
    /// Get PID gains service
    bool getPidGains(dynamixel::GetPidGains::Request& request,
      dynamixel::GetPidGains::Response& response);
    /// Set compliance service
    bool setCompliance(dynamixel::SetCompliance::Request& request,
      dynamixel::SetCompliance::Response& response);
    /// Get compliance service
    bool getCompliance(dynamixel::GetCompliance::Request& request,
      dynamixel::GetCompliance::Response& response);
    /// Set angle limits service
    bool setAngleLimits(dynamixel::SetAngleLimits::Request& request,
      dynamixel::SetAngleLimits::Response& response);
    /// Get angle limits service
    bool getAngleLimits(dynamixel::GetAngleLimits::Request& request,
      dynamixel::GetAngleLimits::Response& response);
    /// Set max torque service
    bool setMaxTorque(dynamixel::SetMaxTorque::Request& request,
      dynamixel::SetMaxTorque::Response& response);
    /// Get max torque service
    bool getMaxTorque(dynamixel::GetMaxTorque::Request& request,
      dynamixel::GetMaxTorque::Response& response);
    /// Set torque enable service
    bool setTorqueEnable(dynamixel::SetTorqueEnable::Request& request,
      dynamixel::SetTorqueEnable::Response& response);
    /// Get torque enable service
    bool getTorqueEnable(dynamixel::GetTorqueEnable::Request& request,
      dynamixel::GetTorqueEnable::Response& response);
    /// Set torque control mode enable service
    bool setTorqueControlModeEnable(
      dynamixel::SetTorqueControlModeEnable::Request& request,
      dynamixel::SetTorqueControlModeEnable::Response& response);
    /// Get torque control mode enable service
    bool getTorqueControlModeEnable(
      dynamixel::GetTorqueControlModeEnable::Request& request,
      dynamixel::GetTorqueControlModeEnable::Response& response);
    /** @}
      */

    /** \name Private members
      @{
      */
    /// ROS node handle
    ros::NodeHandle nodeHandle_;
    /// Diagnostic updater
    diagnostic_updater::Updater diagnosticUpdater_;
    /// Device name reported by diagnostic engine
    std::string deviceName_;
    /// Acquisition loop rate
    double acquisitionLoopRate_;
    /// Serial port device name
    std::string serialPortDeviceName_;
    /// Serial port baudrate
    int serialPortBaudRate_;
    /// Dynamixel controller
    std::shared_ptr<Controller> controller_;
    /// Retry timeout in case of failure [s]
    double retryTimeout_;
    /// Servo motor ID
    int motorId_;
    /// Time offset
    int64_t timeOffset_;
    /// ROS joint state publisher
    ros::Publisher jointStatePublisher_;
    /// ROS joint state publisher topic name
    std::string jointStatePublisherTopic_;
    /// ROS joint state publisher frame id
    std::string jointStatePublisherFrameId_;
    /// ROS joint state publisher queue size
    int jointStatePublisherQueueSize_;
    /// ROS joint state publisher frequency diagnostic
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic>
      jointStatePublisherFrequencyDiagnostic_;
    /// ROS joint state publisher frequency tolerance percentage
    double jointStatePublisherFreqTolPercentage_;
    /// ROS joint state publisher minimum frequency
    double jointStatePublisherMinFrequency_;
    /// ROS joint state publisher maximum frequency
    double jointStatePublisherMaxFrequency_;
    /// ROS tf broadcaster
    tf::TransformBroadcaster transformBroadcaster_;
    /// Servo model number
    uint16_t modelNumber_;
    /// Servo model name
    std::string modelName_;
    /// Servo firmware version
    uint8_t firmwareVersion_;
    /// Return delay time
    uint16_t returnDelayTime_;
    /// Motor connected
    bool motorConnected_;
    /// Maximum number of ticks
    uint16_t maxTicks_;
    /// Range in degrees
    double rangeInDegrees_;
    /// Rpm per tick
    double rpmPerTick_;
    /// Motor baud rate
    uint8_t servoBaudRate_;
    /// Clockwise angle limit
    double cwAngleLimit_;
    /// Counterclockwise angle limit
    double ccwAngleLimit_;
    /// Highest limit temperature
    uint8_t highestLimitTemperature_;
    /// Highest limit voltage
    double highestLimitVoltage_;
    /// Lowest limit voltage
    double lowestLimitVoltage_;
    /// Maximum torque in percent
    double maxTorque_;
    /// Torque enabled
    bool torqueEnabled_;
    /// Current position
    double currentPosition_;
    /// Goal position
    double goalPosition_;
    /// Moving speed
    double movingSpeed_;
    /// Torque limit in percent
    double torqueLimit_;
    /// P gain
    double pGain_;
    /// I gain
    double iGain_;
    /// D gain
    double dGain_;
    /// Clockwise compliance margin
    uint8_t cwCompliangeMargin_;
    /// Counterclockwise compliance margin
    uint8_t ccwCompliangeMargin_;
    /// Clockwise compliance slope
    uint8_t cwCompliangeSlope_;
    /// Counterclockwise compliance slope
    uint8_t ccwCompliangeSlope_;
    /// Torque control mode enabled
    bool torqueControlModeEnabled_;
    /// Set PID gains service
    ros::ServiceServer setPidGainsService_;
    /// Get PID gains service
    ros::ServiceServer getPidGainsService_;
    /// Set compliance service
    ros::ServiceServer setComplianceService_;
    /// Get compliance service
    ros::ServiceServer getComplianceService_;
    /// Set angle limits service
    ros::ServiceServer setAngleLimitsService_;
    /// Get angle limits service
    ros::ServiceServer getAngleLimitsService_;
    /// Set max torque service
    ros::ServiceServer setMaxTorqueService_;
    /// Get max torque service
    ros::ServiceServer getMaxTorqueService_;
    /// Set torque enable service
    ros::ServiceServer setTorqueEnableService_;
    /// Get torque enable service
    ros::ServiceServer getTorqueEnableService_;
    /// Set torque control mode enable service
    ros::ServiceServer setTorqueControlModeEnableService_;
    /// Get torque control mode enable service
    ros::ServiceServer getTorqueControlModeEnableService_;
    /** @}
      */

  };

}

#endif // DYNAMIXEL_NODE_H
