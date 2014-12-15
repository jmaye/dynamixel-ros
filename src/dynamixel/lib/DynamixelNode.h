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
    /// Calibrate the timing between host PC and Dynamixel device
    void calibrateTime();
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
    /// Time calibration requested
    bool calibrateTime_;
    /// Time calibration number of packets to exchange
    int calibrateTimeNumPackets_;
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
    /** @}
      */

  };

}

#endif // DYNAMIXEL_NODE_H
