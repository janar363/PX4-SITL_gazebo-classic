/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_wind_plugin.h"
#include "common.h"

#include "../include/data_processor/wind_data_processor.h"
#include <vector>

namespace gazebo {

std::vector<gazebo::physics::ModelPtr> models;
std::vector<WindDataProcessor::Position> dronePositions;
WindDataProcessor::Array3D arr("../include/wind_data_test/wisp_50.csv", "../include/wind_data_test/3darr.bin", -25, 25, -25, 25, 0, 10);

GazeboWindPlugin::~GazeboWindPlugin() {
  update_connection_->~Connection();
}

void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
  world_ = world;

  // Get the list of model names from the SDF element
  std::vector<std::string> modelNames;
  if (sdf->HasElement("typhoon_h480")) {
    gazebo::physics::ModelPtr model = world_->ModelByName("typhoon_h480");
    modelNames.push_back("typhoon_h480");
    models.push_back(model);

    gzlog << "single drone found in the world." << std::endl;
    std::cout << "single drone found in the world." << std::endl;
  } else if(sdf->HasElement("typhoon_h480_0")) {
    gzlog << "multiple drones found in the world." << std::endl;
    std::cout << "multiple drones found in the world." << std::endl;
    // Searches for drone models with the prefix "typo_h480_X" 
    int droneIndex = 0;
    while (true) {
      std::string modelName = "typhoon_h480_" + std::to_string(droneIndex);
      gazebo::physics::ModelPtr model = world_->ModelByName(modelName);
      if (model) {
        modelNames.push_back(modelName);
        models.push_back(model);

        droneIndex++;
      } else {
        break;
      }
    }
  } else {
      gzlog << "Model 'typhoon_h480' not found in the world." << std::endl;
      std::cout << "Model 'typhoon_h480' not found in the world." << std::endl;
      return;
  }

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);


  getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  double pub_rate = 2.0;
  getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); //Wind topic publishing rates
  pub_interval_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;
  getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
  // Get the wind params from SDF.
  getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
  getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
  getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
  getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
  getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

  wind_direction_mean_.Normalize();
  wind_gust_direction_mean_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  // Set random wind velocity mean and standard deviation
  wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
  // Set random wind direction mean and standard deviation
  wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
  // Set random wind gust velocity mean and standard deviation
  wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
  // Set random wind gust direction mean and standard deviation
  wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
#endif

}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif
  if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
    gzlog << "exiting on update" << std::endl;
    return;
  }
  last_time_ = now;
  gzlog << "entered on update" << std::endl;

  // on update : 
  if(models.size()){
    std::vector<ignition::math::Vector3d> windValues(models.size());
    for(int i=0; i<models.size(); i++){
      // fetch drone positions
      ignition::math::Pose3d pose = models[i]->WorldPose();
      ignition::math::Vector3d position = pose.Pos();

      if(dronePositions.size() == i){
        dronePositions.push_back((WindDataProcessor::Position){(int)position.X(), (int)position.Y(), (int)position.Z()});
      }

      if(arr.dronePosOffsets.size() == 0 || abs(position.X() - arr.dronePosOffsets[i].x) < 5 || abs(position.Y() - arr.dronePosOffsets[i].y) < 5 || abs(position.Z() - arr.dronePosOffsets[i].z) < 5)
        arr.computePointsSerial3DArray(dronePositions, 21);

      WindDataProcessor::WindVal windVal  = arr.getCubeWindValue(i, position.X(), position.Y(), position.Z());
      windValues[i] = ignition::math::Vector3d(windVal.u, windVal.v, windVal.w);

      // air density at sea level at 15 degree C = 1.225 kg/m^3
      double airDensity = 1.225;
      // drag coeff perpendicular to axis
      double dragCoeff = 1.2;
      /*
         dimensions of Typhoon480 drone assuming it to be as rough cylinder,
         with diameter = w

      */
      double diameter = 0.52; // in meters
      double height = 0.21; // in meters
      // wind pressure = 1/2 * air density * wind velocity ^ 2
      // double windPressure = 0.5 * airDensity * windValues[i].Dot(windValues[i]) * windValues.Normalize();

      // area
      double pi = 3.14;
      double area = height * pi * diameter / 2;

      // wind force = area * wind pressure * dragCoeff
      ignition::math::Vector3d windForce = 0.5 * dragCoeff * airDensity * area * windValues[i].Dot(windValues[i]) * windValues[i].Normalize();

      gazebo::physics::LinkPtr link = models[i]->GetLink("base_link");
      link->AddForce(windForce);
      gzlog << "applying wind force at pos : (" << position.X() << ", " << position.Y() << ", " << position.Z() << ") -> force (" << windForce.X() << ", " << windForce.Y() << ", " << windForce.Z() << std::endl;
      std::cout << "applying wind force at pos : (" << position.X() << ", " << position.Y() << ", " << position.Z() << ") -> force (" << windForce.X() << ", " << windForce.Y() << ", " << windForce.Z() << std::endl; 

      // ignition::math::Vector3d wind_gust(0, 0, 0);
      // // Calculate the wind gust velocity.
      // if (now >= wind_gust_start_ && now < wind_gust_end_) {
      //   // Get normal distribution wind gust strength
      //   double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
      //   wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_ : wind_gust_strength;
      //   // Get normal distribution wind gust direction
      //   ignition::math::Vector3d wind_gust_direction;
      //   wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
      //   wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
      //   wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
      //   wind_gust = wind_gust_strength * wind_gust_direction;
      // }

      gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
      // wind_v->set_x(wind.X() + wind_gust.X());
      // wind_v->set_y(wind.Y() + wind_gust.Y());
      // wind_v->set_z(wind.Z() + wind_gust.Z());
      wind_v->set_x(windValues[i].X());
      wind_v->set_y(windValues[i].Y());
      wind_v->set_z(windValues[i].Z());


      wind_msg.set_frame_id(frame_id_);
      wind_msg.set_time_usec(now.Double() * 1e6);
      wind_msg.set_allocated_velocity(wind_v);

      wind_pub_->Publish(wind_msg);


    }
    

  } else {
    std::cout << "model not found\n";
    gzlog << "model not found\n";
  }
}

GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);
}