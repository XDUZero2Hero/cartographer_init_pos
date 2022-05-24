/**
 * @file icra_init_carto_node.cpp
 * @author your name (you@domain.com)
 * @brief 初始化cartographer位姿
 * @version 0.1
 * @date 2022-05-23
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>

#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"

#include <std_srvs/Trigger.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icra_init_carto_node");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    std::string configuration_basename, configuration_directory;
    if (!private_nh.getParam("configuration_basename", configuration_basename))
    {
        ROS_ERROR("configuration_basename parameters do not exists");
    }

    if (!private_nh.getParam("configuration_directory", configuration_directory))
    {
        ROS_ERROR("configuration_directory parameters do not exists");
    }


    double init_pos_x = 0.0;
    double init_pos_y = 0;
    double init_pos_z = 0;

    double init_quat_x = 0.0;
    double init_quat_y = 0;
    double init_quat_z = 0;
    double init_quat_w = 1;

    private_nh.getParam("init_pos_x",init_pos_x);
    private_nh.getParam("init_pos_y",init_pos_x);
    private_nh.getParam("init_pos_z",init_pos_x);

    private_nh.getParam("init_quat_x",init_quat_x);
    private_nh.getParam("init_quat_y",init_quat_y);
    private_nh.getParam("init_quat_z",init_quat_z);
    private_nh.getParam("init_quat_w",init_quat_w);


    ros::ServiceClient finish_trajectory_client = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    ros::ServiceClient start_trajectory_client = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");


    cartographer_ros_msgs::FinishTrajectory finish_trajectory_request;
    cartographer_ros_msgs::StartTrajectory start_trajectory_request;

    finish_trajectory_request.request.trajectory_id = 1;

    start_trajectory_request.request.use_initial_pose = true;

    start_trajectory_request.request.initial_pose.orientation.w = init_quat_w;
    start_trajectory_request.request.initial_pose.orientation.z = init_quat_z;
    start_trajectory_request.request.initial_pose.orientation.y = init_quat_y;
    start_trajectory_request.request.initial_pose.orientation.x = init_quat_x;

    start_trajectory_request.request.initial_pose.position.x = init_pos_x;
    start_trajectory_request.request.initial_pose.position.y = init_pos_y;
    start_trajectory_request.request.initial_pose.position.z = init_pos_z;

    start_trajectory_request.request.configuration_basename = configuration_basename;
    start_trajectory_request.request.configuration_directory = configuration_directory;

    ros::Rate fail_rate = ros::Rate(1);
    while (!finish_trajectory_client.exists()&&ros::ok())
    {
        ROS_ERROR("Finish trajectory service do not exists, will try after 1 s");
        fail_rate.sleep();
    }
    while (!finish_trajectory_client.call(finish_trajectory_request)&&ros::ok())
    {
        ROS_ERROR("Finish trajectory failed, response %s", finish_trajectory_request.response.status.message.c_str());
        fail_rate.sleep();
    }
    ROS_INFO("succeccfully finished last trajectory,response %s", finish_trajectory_request.response.status.message.c_str());

    while (!start_trajectory_client.exists()&&ros::ok())
    {
        ROS_ERROR("Start trajectory service do not exists, will try after 1 s");
        fail_rate.sleep();
    }
    while (!start_trajectory_client.call(start_trajectory_request)&&ros::ok())
    {
        ROS_ERROR("Start trajectory failed, response %s",start_trajectory_request.response.status.message.c_str());
        fail_rate.sleep();
    }
    ROS_INFO("succeccfully Start last trajectory response: %s",start_trajectory_request.response.status.message.c_str());

    return 0;
}