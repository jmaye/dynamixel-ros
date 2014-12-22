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
#include "dynamixel/SetGoalTorque.h"
#include "dynamixel/GetGoalTorque.h"
#include "dynamixel/SetGoalAcceleration.h"
#include "dynamixel/GetGoalAcceleration.h"
#include "dynamixel/SetGoalPosition.h"
#include "dynamixel/GetGoalPosition.h"
#include "dynamixel/SetMovingSpeed.h"
#include "dynamixel/GetMovingSpeed.h"

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
    /// Set goal torque service
    bool setGoalTorque(dynamixel::SetGoalTorque::Request& request,
      dynamixel::SetGoalTorque::Response& response);
    /// Get goal torque service
    bool getGoalTorque(dynamixel::GetGoalTorque::Request& request,
      dynamixel::GetGoalTorque::Response& response);
    /// Set goal position service
    bool setGoalPosition(dynamixel::SetGoalPosition::Request& request,
      dynamixel::SetGoalPosition::Response& response);
    /// Get goal position service
    bool getGoalPosition(dynamixel::GetGoalPosition::Request& request,
      dynamixel::GetGoalPosition::Response& response);
    /// Set moving speed service
    bool setMovingSpeed(dynamixel::SetMovingSpeed::Request& request,
      dynamixel::SetMovingSpeed::Response& response);
    /// Get moving speed service
    bool getMovingSpeed(dynamixel::GetMovingSpeed::Request& request,
      dynamixel::GetMovingSpeed::Response& response);
    /// Set goal acceleration service
    bool setGoalAcceleration(dynamixel::SetGoalAcceleration::Request& request,
      dynamixel::SetGoalAcceleration::Response& response);
    /// Get goal acceleration service
    bool getGoalAcceleration(dynamixel::GetGoalAcceleration::Request& request,
      dynamixel::GetGoalAcceleration::Response& response);
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
    uint8_t returnDelayTime_;
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
    uint16_t cwAngleLimit_;
    /// Counterclockwise angle limit
    uint16_t ccwAngleLimit_;
    /// Highest limit temperature
    uint8_t highestLimitTemperature_;
    /// Highest limit voltage
    uint8_t highestLimitVoltage_;
    /// Lowest limit voltage
    uint8_t lowestLimitVoltage_;
    /// Maximum torque ratio
    uint16_t maxTorque_;
    /// Torque enabled
    bool torqueEnabled_;
    /// P gain
    uint8_t pGain_;
    /// I gain
    uint8_t iGain_;
    /// D gain
    uint8_t dGain_;
    /// Clockwise compliance margin
    uint8_t cwComplianceMargin_;
    /// Counterclockwise compliance margin
    uint8_t ccwComplianceMargin_;
    /// Clockwise compliance slope
    uint8_t cwComplianceSlope_;
    /// Counterclockwise compliance slope
    uint8_t ccwComplianceSlope_;
    /// Goal position
    uint16_t goalPosition_;
    /// Moving speed
    uint16_t movingSpeed_;
    /// Torque limit ratio
    uint16_t torqueLimit_;
    /// Current position
    uint16_t presentPosition_;
    /// Current speed
    uint16_t presentSpeed_;
    /// Current load
    uint16_t presentLoad_;
    /// Current voltage
    uint8_t presentVoltage_;
    /// Current temperature
    uint8_t presentTemperature_;
    /// Is an instruction registered
    bool registered_;
    /// Is the servo moving
    bool moving_;
    /// Punch
    uint16_t punch_;
    /// Consuming current
    uint16_t current_;
    /// Torque control mode enabled
    bool torqueControlModeEnabled_;
    /// Goal torque
    uint16_t goalTorque_;
    /// Goal acceleration
    uint8_t goalAcceleration_;
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
    /// Set goal torque service
    ros::ServiceServer setGoalTorqueService_;
    /// Get goal torque service
    ros::ServiceServer getGoalTorqueService_;
    /// Set goal position service
    ros::ServiceServer setGoalPositionService_;
    /// Get goal position service
    ros::ServiceServer getGoalPositionService_;
    /// Set moving speed service
    ros::ServiceServer setMovingSpeedService_;
    /// Get moving speed service
    ros::ServiceServer getMovingSpeedService_;
    /// Set goal acceleration service
    ros::ServiceServer setGoalAccelerationService_;
    /// Get goal acceleration service
    ros::ServiceServer getGoalAccelerationService_;
    /** @}
      */

  };

}

#endif // DYNAMIXEL_NODE_H
