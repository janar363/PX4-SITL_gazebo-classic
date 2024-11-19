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
    std::vector<WindDataProcessor::Point> dronePositions;
    static std::unique_ptr<WindDataProcessor::WindDataProcessor> windProcessor;

    GazeboWindPlugin::~GazeboWindPlugin() {
        update_connection_->~Connection();
    }

    void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
        world_ = world;
        sdf_ = sdf;  // Store the SDF pointer

        // Load wind data file
        const std::string windDataFile = sdf->Get<std::string>("windDataFile", "wisp_50.csv").first;
        windProcessor = std::make_unique<WindDataProcessor::WindDataProcessor>(windDataFile);

        // Detect and load models
        if (!LoadModels()) {
            gzerr << "Failed to find required models in the world. Retrying..." << std::endl;
            retry_load_connection_ = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&GazeboWindPlugin::RetryLoad, this));
            return;
        }

        // Initialize the plugin
        this->InitPlugin(sdf);
    }

    bool GazeboWindPlugin::LoadModels() {
        if (sdf_->HasElement("typhoon_h480")) {
            return LoadSingleModel("typhoon_h480");
        } else if (sdf_->HasElement("typhoon_h480_0")) {
            return LoadMultipleModels("typhoon_h480_");
        } else {
            gzerr << "No valid model configuration found in the SDF file." << std::endl;
            return false;
        }
    }

    bool GazeboWindPlugin::LoadSingleModel(const std::string &modelName) {
        auto model = world_->ModelByName(modelName);
        if (model) {
            models.push_back(model);
            gzmsg << "Single drone model '" << modelName << "' found in the world." << std::endl;
            return true;
        } else {
            gzerr << "Model '" << modelName << "' not found in the world." << std::endl;
            return false;
        }
    }

    bool GazeboWindPlugin::LoadMultipleModels(const std::string &modelPrefix) {
        int droneIndex = 0;
        while (true) {
            const std::string modelName = modelPrefix + std::to_string(droneIndex);
            auto model = world_->ModelByName(modelName);
            if (model) {
                models.push_back(model);
                droneIndex++;
            } else {
                break;
            }
        }

        if (!models.empty()) {
            gzmsg << "Multiple drone models with prefix '" << modelPrefix << "' found in the world." << std::endl;
            return true;
        } else {
            gzerr << "No drone models with prefix '" << modelPrefix << "' found in the world." << std::endl;
            return false;
        }
    }


    void GazeboWindPlugin::RetryLoad() {
        if (LoadModels()) {
            retry_load_connection_.reset();
            InitPlugin(sdf_);
        }
    }

    void GazeboWindPlugin::InitPlugin(sdf::ElementPtr sdf) {
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
//        gzerr << "wind plugin on update called" << std::endl;
        // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
#else
        common::Time now = world_->GetSimTime();
#endif
        if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
            return;
        }
        last_time_ = now;

        // on update :
        if(!models.empty()){
            size_t i = 0;
            std::vector<ignition::math::Vector3d> windValues(models.size());
            for (auto &model : world_->Models()) {
                if (i >= windValues.size()) {
                    windValues.emplace_back(); // Ensure windValues is large enough
                }

                // Fetch drone positions
                auto pose = model->WorldPose().Pos();
                auto wind = getNearestWindValue(pose.X(), pose.Y(), pose.Z());

                // Update windValues for the current model
                windValues[i] = ignition::math::Vector3d(wind.u, wind.v, wind.w);

                // Calculate wind force
                ignition::math::Vector3d windForce = windValues[i].Dot(windValues[i]) * windValues[i].Normalize();

                // Apply the force to the link
                gazebo::physics::LinkPtr link = model->GetLink("base_link");
                if (link) {
                    link->SetForce(windForce);
                }



                gzlog << "applying wind force at pos : (" << pose.X() << ", " << pose.Y() << ", " << pose.Z() << ") -> force (" << windForce.X() << ", " << windForce.Y() << ", " << windForce.Z() << std::endl;
                std::cout << "applying wind force at pos : (" << pose.X() << ", " << pose.Y() << ", " << pose.Z() << ") -> force (" << windForce.X() << ", " << windForce.Y() << ", " << windForce.Z() << std::endl;


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
                ++i; // Increment index for the next iteration
            }
        } else {
            std::cout << "model not found\n";
            gzlog << "model not found\n";
        }
    }

    GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);
}
