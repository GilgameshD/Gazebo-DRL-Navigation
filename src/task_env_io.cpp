#define _USE_MATH_DEFINES

#include <iostream>
#include <ignition/math.hh>
#include <assert.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <time.h>
#include <mutex>
#include <ctime>
#include <random>

#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>  // roslogging
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <typeinfo>

#include "gazebo_env_io.h"
#include "task_env_io.h"


//protect the read and write for topic vectors
std::mutex topic_mutex;
#define PI 3.14159265359
#define ROBOT_NEAR_DISTANCE_X 0.5
#define ROBOT_NEAR_DISTANCE_Y 0.5

// state callback function, with mutex lock because of multiprocessing
template<typename topicType>
void RL::GetNewTopic<topicType>::StateCallback(const topicType& msg_)
{
    std::lock_guard<std::mutex> lock(topic_mutex);
    stateVector.push_back(msg_);
    // the max state is 5
    if (stateVector.size() > 5)
        stateVector.erase(stateVector.begin());
}

template<typename topicType>
RL::GetNewTopic<topicType>::GetNewTopic(ros::NodeHandlePtr rosNode_pr, const std::string topicName)
{
    StateSub = rosNode_pr->subscribe(topicName, 1, &GetNewTopic::StateCallback, this);
}

RL::ParamLoad::ParamLoad(ros::NodeHandlePtr rosNode_pr_): rosNodeConstPtr(rosNode_pr_)
{
    assert(rosNodeConstPtr->getParam("/MAP_TYPE", mapType));
    assert(rosNodeConstPtr->getParam("/AGENT_NUMBER", agentNumber));
    assert(rosNodeConstPtr->getParam("/STATIC_COLLISION_TH", staticCollisionThreshold));
    assert(rosNodeConstPtr->getParam("/DYNAMIC_COLLISION_TH", dynamicCollisionThreshold));
    assert(rosNodeConstPtr->getParam("/STATIC_COLLISION_REWARD", staticCollisionReward));
    assert(rosNodeConstPtr->getParam("/DYNAMIC_COLLISION_REWARD", dynamicCollisionReward));
    assert(rosNodeConstPtr->getParam("/TERMINAL_REWARD", terminalReward));
    assert(rosNodeConstPtr->getParam("/SURVIVE_REWARD", surviveReward));
    assert(rosNodeConstPtr->getParam("/EDGE_REWARD", edgeReward));
    assert(rosNodeConstPtr->getParam("/TARGET_TH", targetThreshold));

    assert(rosNodeConstPtr->getParam("/MAX_LINEAR_VAL", maxLinearVel));
    assert(rosNodeConstPtr->getParam("/MAX_ANGULAR_VAL", maxAngularVel));

    assert(rosNodeConstPtr->getParam("/ENABLE_TARGET_TERMINAL", enableTargetTerminal));
    assert(rosNodeConstPtr->getParam("/ENABLE_STATIC_COLLISIOM_TERMINAL", enableStaticCollisionTerminal));
    assert(rosNodeConstPtr->getParam("/ENABLE_DYNAMIC_COLLISIOM_TERMINAL", enableDynamicCollisionTerminal));
    assert(rosNodeConstPtr->getParam("/ENABLE_LASER_DETECTION", enableLaserDetection));
    assert(rosNodeConstPtr->getParam("/ENABLE_EDGE_DETECTION", enableEdgeDetection));
    assert(rosNodeConstPtr->getParam("/ENABLE_SURVIVE_REWARD", enableSurviveReward));

    assert(rosNodeConstPtr->getParam("/MAP_MAX_RANGE", map_max_range));
    assert(rosNodeConstPtr->getParam("/MAP_MIN_RANGE", map_min_range));
    assert(rosNodeConstPtr->getParam("/TARGET_X_START", target_x_start));
    assert(rosNodeConstPtr->getParam("/TARGET_X_END",   target_x_end));
    assert(rosNodeConstPtr->getParam("/TARGET_Y_START", target_y_start));
    assert(rosNodeConstPtr->getParam("/TARGET_Y_END",   target_y_end));
    assert(rosNodeConstPtr->getParam("/ROBOT_X_START", robot_x_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_X_END",   robot_x_end));
    assert(rosNodeConstPtr->getParam("/ROBOT_Y_START", robot_y_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_Y_END",   robot_y_end));
    assert(rosNodeConstPtr->getParam("/ROBOT_YAW_START", robot_yaw_start));
    assert(rosNodeConstPtr->getParam("/ROBOT_YAW_END",   robot_yaw_end));
}

// construction function for class TaskEnvIO
// in this function, all parameters and topics are defined
RL::TaskEnvIO::TaskEnvIO(const std::string serviceName, const std::string nodeName):
GazeboEnvIO(nodeName),
paramList(new RL::ParamLoad(this->rosNode_pr)),
logFileName("/home/dwh/1_normal_workspace/catkin_ws/src/gazebo_drl_env/log/logger.txt")
{
    // generate robot name
    this->AGENT_NUM = paramList->agentNumber;
    this->mapType = paramList->mapType;
    GenerateRobotNames(this->AGENT_NUM, RL::ROBOT_NAMES_BASE, ROBOT_NAMES, TARGET_NAMES);

    // position generator
    positionGenerator = new PositionGenerator(this->AGENT_NUM, paramList->robot_yaw_start, paramList->robot_yaw_end);

    // init terminal flag
    terminalFlag = new bool[this->AGENT_NUM]();

    // output logger head
    std::fstream fileOperator(this->logFileName, std::ios::out | std::ios::app);
    fileOperator << "-------------------- [" << ros::WallTime::now() << "] --------------------" << std::endl;
    fileOperator.close();

    // define publishers, in order to control the robot (turtlebot)
    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        ros::Publisher newActionPub;
        newActionPub = this->rosNode_pr->advertise<RL::ACTION_TYPE>(StringCat(std::string("/"), ROBOT_NAMES[i], std::string("/cmd_vel")), 1);
        std::shared_ptr<RL::GetNewTopic<RL::STATE_TYPE_2>> newLaserScan;
        newLaserScan = std::make_shared<RL::GetNewTopic<RL::STATE_TYPE_2>>(this->rosNode_pr, StringCat(std::string("/"), ROBOT_NAMES[i], std::string("/scan")));

        actionPublishers.push_back(newActionPub);
        laserScans.push_back(newLaserScan);

        // initialize target position and survive reward
        ignition::math::Vector3d currentTargetPose(0,0,0);
        targetPose.push_back(currentTargetPose);
        totalSurviveReward.push_back(0.0);
    }

    modelState = std::make_shared<RL::GetNewTopic<gazebo_msgs::ModelStates>>(this->rosNode_pr, "/gazebo/model_states");

    // define a service which interact with pytorch
    PytorchService = this->rosNode_pr->advertiseService(serviceName, &TaskEnvIO::ServiceCallback, this);

    // define two states that are related to model state
    SetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
    GetModelPositionClient = this->rosNode_pr->serviceClient<gazebo_msgs::SetModelState>("/gazebo/get_model_state"); 
}

// Set a separate callbackqueue for this service callback 
// Otherwise, the other callbacks will not work when there is a reset loop
bool RL::TaskEnvIO::ServiceCallback(gazebo_drl_env::SimpleCtrl::Request &req, gazebo_drl_env::SimpleCtrl::Response &res)
{
    // check every reset flag and publish actions
    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        if (req.all_group_controls.group_control[i].reset)
        {
            this->ResetOneAgent(ROBOT_NAMES[i]);
            ROS_ERROR("[RESET ONE AGENT] -- [%d]", i);
        }
        else
            ActionPub(req.all_group_controls.group_control[i].linear_x, req.all_group_controls.group_control[i].angular_z, ROBOT_NAMES[i]);
    }

    // check model state
    std::unique_lock<std::mutex> modelStateLock(topic_mutex);
    assert(modelState->stateVector.size() > 0);
    this->newStates = modelState->stateVector.back();
    modelStateLock.unlock();

    // return current position
    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        ignition::math::Pose3d robotState = RL::GazePose2IgnPose(FindPosebyName(ROBOT_NAMES[i]));

        gazebo_drl_env::state_msgs tempState;
        // return current position
        tempState.current_x = robotState.Pos().X();
        tempState.current_y = robotState.Pos().Y();
        // return target position
        tempState.target_x = targetPose[i].X();
        tempState.target_y = targetPose[i].Y();

        // return desired forces
        ignition::math::Vector3d desired_force = targetPose[i]-robotState.Pos();
        ignition::math::Angle desired_angle = std::atan2(desired_force.Y(), desired_force.X())-robotState.Rot().Yaw();
        desired_angle.Normalize();
        tempState.desired_x = std::cos(desired_angle.Radian());
        tempState.desired_y = std::sin(desired_angle.Radian());
        res.all_group_states.group_state.push_back(tempState);

        // build image state
        //std::unique_lock<std::mutex> front_image_lock(topic_mutex);
        //assert(front_image_1->stateVector.size()>0);
        //res.all_group_states.group_state[i].depth_img = *(front_image_1->stateVector.back());
        //front_image_lock.unlock();

        // collision detection
        std::unique_lock<std::mutex> laserScanLock(topic_mutex);
        assert(laserScans[i]->stateVector.size()>0);
        laserData = *(laserScans[i]->stateVector.back());
        laserScanLock.unlock();

        // return laser data, only use float32 data
        res.all_group_states.group_state[i].laserScan = laserData.ranges;

        // use laser data to check collision, not conducted in reset mode
        bool laserCollisionFlag = false;
        if (paramList->enableStaticCollisionTerminal == true)
        {
            for (int j = 0;j < 360; j++)
            {
                if (laserData.ranges[j] < paramList->staticCollisionThreshold && !req.all_group_controls.group_control[i].reset)
                {
                    laserCollisionFlag = true;
                    break;
                }
            }
        }

        // reward calculation
        res.all_group_states.group_state[i].reward = RewardCalculate(robotState, laserCollisionFlag, i);

        // if get to the terminal, stop the robot
        res.all_group_states.group_state[i].terminal = this->terminalFlag[i];

        if (res.all_group_states.group_state[i].terminal && !req.all_group_controls.group_control[i].reset)
            ActionPub(0.0, 0.0, ROBOT_NAMES[i]);
        // flip the terminal flag
        this->terminalFlag[i] = false;
    }
    return true;
}

bool RL::TaskEnvIO::ActionPub(const float linear_x, const float angular_z, const std::string modelName)
{
    geometry_msgs::Twist actionOut;
    // calculate the final speed of linear and angular
    // in this environment, we set only two var, but totaly, there are 6DOF vars
    actionOut.linear.x = linear_x * paramList->maxLinearVel;
    actionOut.angular.z = angular_z * paramList->maxAngularVel;  
    
    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        if (modelName == ROBOT_NAMES[i])
        {
            actionPublishers[i].publish(actionOut);
            return true;
        }
    }
    ROS_ERROR("[NO SUCH A ROBOT NAME]");
    return false;
}

// notice that we only have a reward when we get to the target
float RL::TaskEnvIO::RewardCalculate(const ignition::math::Pose3d robotIgnPose, const bool laserCollisionFlag, const int agentNumber)
{
    // all these checks have priority
    if (EdgeCheck(robotIgnPose) && paramList->enableEdgeDetection)
    {
        this->terminalFlag[agentNumber] = paramList->enableEdgeDetection;
        return paramList->edgeReward;
    }
    else if (TargetCheck(robotIgnPose, agentNumber) && paramList->enableTargetTerminal)
    {
        this->terminalFlag[agentNumber] = paramList->enableTargetTerminal;
        return paramList->terminalReward;
    }
    else if (StaticCollisionCheck(robotIgnPose, laserCollisionFlag) && paramList->enableStaticCollisionTerminal)
    {
        this->terminalFlag[agentNumber] = paramList->enableStaticCollisionTerminal;
        return paramList->staticCollisionReward;
    }
    else if (DynamicCollisionCheck(robotIgnPose, agentNumber) && paramList->enableDynamicCollisionTerminal)
    {
        this->terminalFlag[agentNumber] = paramList->enableDynamicCollisionTerminal;
        return paramList->dynamicCollisionReward;
    }
    else if (paramList->enableSurviveReward)
    {
        this->totalSurviveReward[agentNumber] += paramList->surviveReward;
        this->terminalFlag[agentNumber] = false;
        return totalSurviveReward[agentNumber];
    }
    else
    {
        this->terminalFlag[agentNumber] = false;
        return 0.0;
    }
}

bool RL::TaskEnvIO::ResetOneAgent(std::string robotName) 
{
    geometry_msgs::Twist actionOut;
    actionOut.angular.z = 0;
    actionOut.linear.x = 0;
    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        if (ROBOT_NAMES[i] == robotName)
            actionPublishers[i].publish(actionOut);
    }

    std::unique_lock<std::mutex> modelStateLock(topic_mutex);
    assert(modelState->stateVector.size()>0);
    this->newStates = modelState->stateVector.back();
    modelStateLock.unlock();

    float randomX, randomY;
    geometry_msgs::Quaternion randomQ;

    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        if (ROBOT_NAMES[i] == robotName)
        {
            // generate position for one agent randomly
            if (this->mapType == 1)
                positionGenerator->Mode_1_Agent_8_FourBisection_DiagonalTarget(i);
            else if (this->mapType == 3)
                positionGenerator->Mode_3_Agent_16_NoTarget(i);
            else if (this->mapType == 4)
                positionGenerator->Mode_4_Agent_1_RandomTarget(i);
            else
                ROS_ERROR("No such map");

            // reset agent model
            positionGenerator->ReturnAgentPositionByIndex(randomX, randomY, randomQ, i);
            SetModelPosition(randomX, randomY, randomQ, ROBOT_NAMES[i]);

            // reset new targte
            positionGenerator->ReturnTargetPositionByIndex(randomX, randomY, randomQ, i);
            targetPose[i].X() = randomX;
            targetPose[i].Y() = randomY;
            SetModelPosition(randomX, randomY, randomQ, TARGET_NAMES[i]);
            this->totalSurviveReward[i] = 0;
        }
    }
    return true;
}

geometry_msgs::Pose RL::TaskEnvIO::FindPosebyName(const std::string model_name_) const 
{
    geometry_msgs::Pose modelState;
    auto idx_ = std::find(this->newStates.name.begin(), this->newStates.name.end(), model_name_) - this->newStates.name.begin();
    modelState = this->newStates.pose.at(idx_);
    return modelState;
}

bool RL::TaskEnvIO::SetModelPosition(const float x, const float y, const geometry_msgs::Quaternion q, const std::string modelName)
{
    gazebo_msgs::SetModelState SetModelState_srv;
    geometry_msgs::Point newPosition;
    newPosition.x = x;
    newPosition.y = y;
    newPosition.z = 0.03;  // z position is alawys zero

    geometry_msgs::Pose newPose;
    newPose.position = newPosition;
    newPose.orientation = q;

    gazebo_msgs::ModelState newModelstate;
    newModelstate.model_name = modelName;
    newModelstate.pose = newPose;

    SetModelState_srv.request.model_state = newModelstate;
    return SetModelPositionClient.call(SetModelState_srv);
}

bool RL::TaskEnvIO::TargetCheck(const ignition::math::Pose3d robotPose, const int agentNumber) const
{
    float target_distance = (targetPose[agentNumber]-robotPose.Pos()).Length();
    return (target_distance < paramList->targetThreshold)? true : false;
}

bool RL::TaskEnvIO::StaticCollisionCheck(const ignition::math::Pose3d robot_pose_, bool laser_collision_flag) const
{ 
    if(laser_collision_flag && paramList->enableLaserDetection)
        return true;
    else
        return false;
}

bool RL::TaskEnvIO::DynamicCollisionCheck(const ignition::math::Pose3d robot_pose_, const int agentNumber) const
{   
    for (int i = 0;i < this->AGENT_NUM;i++)
    {
        if (i == agentNumber)
            continue;
        geometry_msgs::Pose robotState = FindPosebyName(ROBOT_NAMES[i]);
        ignition::math::Pose3d tempState = RL::GazePose2IgnPose(robotState);
        float distance = (tempState.Pos()-robot_pose_.Pos()).Length();
        if (distance <= paramList->dynamicCollisionThreshold)
            return true;
    }
    return false;
}

bool RL::TaskEnvIO::EdgeCheck(const ignition::math::Pose3d robot_pose_) const
{
    if ((robot_pose_.Pos()[0] > paramList->map_max_range  || 
         robot_pose_.Pos()[0] < paramList->map_min_range  || 
         robot_pose_.Pos()[1] > paramList->map_max_range  || 
         robot_pose_.Pos()[1] < paramList->map_min_range) && paramList->enableEdgeDetection)
        return true;
    else
        return false;
}

bool RL::TaskEnvIO::GenerateRobotNames(int robotNumber, std::string robotNameBase, std::vector<std::string>& robotName, std::vector<std::string>& targetName)
{
    for (int i = 0;i < robotNumber;i++)
    {
        std::string newRobotName = robotNameBase + std::to_string(i+1);
        robotName.push_back(newRobotName);
        targetName.push_back(newRobotName + std::string("_target"));
    }
    if(robotName.empty() || targetName.empty())
        return false;
    else
        return true;
}

bool RL::TaskEnvIO::WriteLog(const float data)
{
    std::fstream fileOperator(this->logFileName, std::ios::out | std::ios::app);
    fileOperator << data << std::endl;
    fileOperator.close();
}

ignition::math::Pose3d RL::GazePose2IgnPose(const geometry_msgs::Pose pose_) 
{
    return ignition::math::Pose3d
    (
        pose_.position.x,
        pose_.position.y,
        pose_.position.z,
        pose_.orientation.w,
        pose_.orientation.x,
        pose_.orientation.y,
        pose_.orientation.z
    );
}

ignition::math::Quaterniond RL::Yaw2Quaterniond(const ignition::math::Angle yaw_)
{
    return ignition::math::Quaterniond(0, 0, yaw_.Radian());
}

const char* RL::StringCat(std::string s1, std::string s2, std::string s3)
{
    s1 += s2;
    s1 += s3;
    return s1.c_str();
}

double RL::QuaternionToYaw(const geometry_msgs::Quaternion &state_quat_) 
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(state_quat_, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return double(yaw);
}