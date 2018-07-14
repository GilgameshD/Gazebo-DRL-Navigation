/*************************************************************************
	> File Name: main.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Di 09 Mai 2017 18:45:03 CEST
 ************************************************************************/

#include <ros/ros.h>
#include "task_env_io.h"
#include "gazebo_env_io.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_env_io");
  RL::TaskEnvIO taskenv("pytorch_io_service", "gazebo_env_io");
  
  ros::AsyncSpinner spinner(8); // use 4 threads
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}

