#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/String.h>

#define ego 1
#define fuel 2

static nav_msgs::Path lala;


static geometry_msgs::Point current_local_objective;

static std_msgs::String uav_status;
void uav_status_callback(const std_msgs::String::ConstPtr &msg)
{
    uav_status = *msg;
}

static nav_msgs::Odometry uav_odom;
static bool got_odom = false;
void odometrycallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_odom = *msg;
    got_odom = true;
}

static trajectory_msgs::MultiDOFJointTrajectoryPoint ego_setpoint;
static bool got_ego_command = false;
void ego_command_callback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg)
{
    // std::cout<<"hgaini"<<std::endl;
    ego_setpoint = *msg;
    // std::cout<<msg->transforms.size()<<std::endl;
    // for(auto what : msg->transforms)
    // {
    //     std::cout<<what.translation.x<<std::endl;
    // }
    // std::cout<<ego_setpoint.transforms[0].translation.x<<std::endl;
    got_ego_command = true;
}

static trajectory_msgs::MultiDOFJointTrajectoryPoint fuel_setpoint;
void fuel_command_callback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg)
{

}

bool infer_poi_reached_or_not(
    nav_msgs::Odometry uav_odom, 
    geometry_msgs::Point current_local_objective
)
{


}

static nav_msgs::Path wp_commmand_to_ego;
void set_wp_command_to_ego(geometry_msgs::PoseStamped destination_point)
{
    // std::cout<<destination_point.pose.position.x<<std::endl;
    // std::cout<<destination_point.pose.position.y<<std::endl;
    // std::cout<<destination_point.pose.position.z<<std::endl<<std::endl;
    wp_commmand_to_ego.poses.clear();
    wp_commmand_to_ego.poses.emplace_back(destination_point);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_fsm_eg");
    ros::NodeHandle nh("~");

    double fsm_x, fsm_y, fsm_z;
    nh.getParam("fsm_x", fsm_x);
    nh.getParam("fsm_y", fsm_y);
    nh.getParam("fsm_z", fsm_z);

    std::cout<<fsm_x<<std::endl;
    std::cout<<fsm_y<<std::endl;
    std::cout<<fsm_z<<std::endl;

    ros::Subscriber uav_odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/red/odometry", 1, odometrycallback);

    ros::Subscriber uav_status_sub = nh.subscribe<std_msgs::String>
        ("/red/carrot/status", 1 , uav_status_callback);

    ros::Subscriber ego_command_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/ego/red/position_hold/trajectory", 1, ego_command_callback);

    ros::Subscriber fuel_command_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/fuel/red/position_hold/trajectory", 1, fuel_command_callback);



    ros::Publisher waypoints_pub = nh.advertise<nav_msgs::Path>
        ("/waypoint_generator/waypoints", 1);

    ros::Publisher command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/red/position_hold/trajectory", 1);
    
    ros::Rate a_fsm_rate_ctrl(100);

    //pre-flight check
    while(ros::ok())
    {
        if(got_odom)
            break;

        std::cout<<uav_status.data<<std::endl;
        std::cout<<got_odom<<std::endl;

        ros::spinOnce();
        a_fsm_rate_ctrl.sleep();        
    }

    geometry_msgs::PoseStamped test_destination;
    test_destination.pose.position.x = fsm_x;
    test_destination.pose.position.y = fsm_y;
    test_destination.pose.position.z = fsm_z;
    // test_destinatio

    std::cout<<test_destination.pose.position.x<<std::endl;
    std::cout<<test_destination.pose.position.y<<std::endl;
    std::cout<<test_destination.pose.position.z<<std::endl;
    
    double last_request = ros::Time::now().toSec();

    while(ros::ok())
    {
        if(
            (ros::Time::now().toSec() - last_request) < 
            ros::Duration(1.0).toSec()
        )
        {
            // std::cout<<"hi"<<std::endl;
            set_wp_command_to_ego(test_destination);
            waypoints_pub.publish(wp_commmand_to_ego);
        }
        
        // std::cout<<got_ego_command<<std::endl;
        if(got_ego_command)
        {
            command_pub.publish(ego_setpoint);
            got_ego_command = false;
        }
            

        ros::spinOnce();
        a_fsm_rate_ctrl.sleep();
        
        // std::cout<<got_ego_command<<std::endl<<std::endl;
    }

    return 0;
}