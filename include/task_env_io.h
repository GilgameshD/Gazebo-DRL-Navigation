#ifndef _TASK_ENV_IO_H
#define _TASK_ENV_IO_H

#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ignition/math.hh>

#include "gazebo_env_io.h"
#include "position_generator.h"


namespace RL 
{
    // typedef
    using STATE_TYPE_1 = sensor_msgs::ImageConstPtr;
    using STATE_TYPE_2 = sensor_msgs::LaserScanConstPtr;
    using ACTION_TYPE = geometry_msgs::Twist;

    const std::string ROBOT_NAMES_BASE = {"agent_"};

    ignition::math::Pose3d GazePose2IgnPose(const geometry_msgs::Pose);
    ignition::math::Quaterniond Yaw2Quaterniond(const ignition::math::Angle);
    const char* StringCat(std::string, std::string, std::string);
    double QuaternionToYaw(const geometry_msgs::Quaternion&);

    // Template class for subscriber 
    template<typename topicType>
    class GetNewTopic
    {
        public:
            GetNewTopic(ros::NodeHandlePtr, const std::string);
            std::vector<topicType> stateVector;
        private:
            void StateCallback(const topicType&);
            ros::Subscriber StateSub; 
    };

    // Paramaters loading Class
    class ParamLoad
    {
        public:
            ParamLoad(ros::NodeHandlePtr);

            int mapType;
            int agentNumber;
            float staticCollisionThreshold;
            float dynamicCollisionThreshold;
            float targetThreshold;
            
            // rewards
            float terminalReward;
            float staticCollisionReward;
            float dynamicCollisionReward;
            float edgeReward;
            float surviveReward;

            float maxLinearVel;
            float maxAngularVel;

            // flags
            bool enableLaserDetection;
            bool enableStaticCollisionTerminal;
            bool enableDynamicCollisionTerminal;
            bool enableEdgeDetection;
            bool enableSurviveReward;
            bool enableTargetTerminal;

            // parameters
            float robot_x_start;
            float robot_x_end;
            float robot_y_start;
            float robot_y_end;
            float robot_yaw_start;
            float robot_yaw_end;

            float target_x_start;
            float target_x_end;
            float target_y_start;
            float target_y_end;
            
            float map_min_range;
            float map_max_range;
        private:
            const ros::NodeHandlePtr rosNodeConstPtr;
    };

    // Main class inherit from gazeboenvio
    class TaskEnvIO: public RL::GazeboEnvIO
    {
        private:
            int mapType;
            int AGENT_NUM;
            std::vector<std::string> ROBOT_NAMES;
            std::vector<std::string> TARGET_NAMES;

            // all control action publisher
            std::vector<ros::Publisher> actionPublishers;

            // all agent state topics
            //std::vector<std::shared_ptr<RL::GetNewTopic<RL::STATE_TYPE_1>>> frontImages;

            // all laser scan topics
            std::vector<std::shared_ptr<RL::GetNewTopic<RL::STATE_TYPE_2>>> laserScans;

            // model state topic, only one, find by their names
            std::shared_ptr<RL::GetNewTopic<gazebo_msgs::ModelStates>> modelState;
            
            ros::ServiceClient SetModelPositionClient;
            ros::ServiceClient GetModelPositionClient;

            bool *terminalFlag;
            std::string logFileName;
            gazebo_msgs::ModelStates newStates;
            sensor_msgs::LaserScan laserData;
            std::vector<ignition::math::Vector3d> targetPose;
            std::vector<float> totalSurviveReward;
            RL::PositionGenerator *positionGenerator;

            // save all parameters
            const std::shared_ptr<ParamLoad> paramList;

            // set x-y-z position of one agent
            bool SetModelPosition(const float, const float, const geometry_msgs::Quaternion, const std::string);

            // check if we have a collosion with static objects
            bool StaticCollisionCheck(const ignition::math::Pose3d, const bool) const;
            
            // check if we have a collosion with dynamic objects
            bool DynamicCollisionCheck(const ignition::math::Pose3d, const int) const;

            // check if we arrive the target
            bool TargetCheck(const ignition::math::Pose3d, const int) const;
            
            // check if get to the edge
            bool EdgeCheck(const ignition::math::Pose3d) const;

            // publish the moving action
            bool ActionPub(const float, const float, const std::string);

            // get current reward
            float RewardCalculate(const ignition::math::Pose3d, const bool, const int);

            // get pose of models
            geometry_msgs::Pose FindPosebyName(const std::string) const;
            
            // generate all 
            bool GenerateRobotNames(const int, const std::string, std::vector<std::string>&, std::vector<std::string>&);

            // define the service callback function
            // in this case, the name of the service is SocialForce, so the class name is it
            virtual bool ServiceCallback(gazebo_drl_env::SimpleCtrl::Request&, gazebo_drl_env::SimpleCtrl::Response&);
            virtual bool ResetOneAgent(const std::string);
            virtual bool WriteLog(const float);

        public: 
            TaskEnvIO(const std::string serviceName="pytorch_io_service", const std::string nodeName="gazebo_env_io");
    };
}

#endif
