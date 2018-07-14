#ifndef _GAZEBO_ENV_IO_H
#define _GAZEBO_ENV_IO_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <thread>
#include <gazebo_drl_env/PytorchRL.h>
#include <gazebo_drl_env/SimpleCtrl.h>
#include <gazebo_drl_env/state_msgs.h>

namespace RL
{
    class GazeboEnvIO
    {
        public:
            ros::NodeHandlePtr rosNode_pr;
            ros::ServiceServer PytorchService;
            
            GazeboEnvIO(const std::string nodeName="gazebo_env_io"):rosNode_pr(new ros::NodeHandle(nodeName)){};
            
            // only use the social force service
            virtual bool ServiceCallback(gazebo_drl_env::SimpleCtrl::Request&, gazebo_drl_env::SimpleCtrl::Response&) =0;
            virtual bool ResetOneAgent(const std::string) =0;
            virtual bool WriteLog(const float) =0;
    };
}

#endif


