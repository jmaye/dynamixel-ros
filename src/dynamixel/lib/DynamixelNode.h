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

#include <ros/ros.h>

namespace dynamixel {

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
    /** @}
      */

    /** \name Private members
      @{
      */
    /// ROS node handle
    ros::NodeHandle nodeHandle_;
    /** @}
      */

  };

}

#endif // DYNAMIXEL_NODE_H
