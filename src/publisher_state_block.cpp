//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------

#include "publisher_state_block.h"

namespace wolf
{

PublisherStateBlock::PublisherStateBlock(const std::string& _unique_name,
                                         const ParamsServer& _server,
                                         ProblemConstPtr _problem) :
        Publisher(_unique_name, _server, _problem),
        msg_init_(false)
{
  sensor_ = _problem->findSensor(_server.getParam<std::string>(prefix_ + "/sensor"));
  assert(sensor_);
  key_ = _server.getParam<char>(prefix_ + "/key");
}

void PublisherStateBlock::initialize(ros::NodeHandle& nh, const std::string& topic)
{
    publisher_ = nh.advertise<std_msgs::Float64MultiArray>(topic, 1);
}

void PublisherStateBlock::publishDerived()
{
  WOLF_WARN_COND(not sensor_->getStateBlock(key_), "StateBlock not found")
  if(not sensor_->getStateBlock(key_))
    return;
  Eigen::VectorXd state_vec = sensor_->getStateBlock(key_)->getState();
  if(not msg_init_)
  {
    std_msgs::MultiArrayDimension dim;
    dim.label = sensor_->getName()+"/"+std::to_string(key_);
    dim.size = state_vec.size();
    dim.stride = state_vec.size();
    state_msg_.layout.dim.push_back(dim);
    state_msg_.layout.data_offset = 0;
    state_msg_.data.resize(state_vec.size());
    msg_init_ = true;
  }
  Eigen::Map<Eigen::VectorXd> msg_map(state_msg_.data.data(), state_vec.size());
  msg_map = state_vec;

  publisher_.publish(state_msg_);

}

WOLF_REGISTER_PUBLISHER(PublisherStateBlock)
}
