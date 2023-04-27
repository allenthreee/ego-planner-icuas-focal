#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/String.h>

#include <plan_env/grid_map.h>

#include <vector>
#include <Eigen/Dense>

#include <tf/transform_datatypes.h>


#define go_up 0
#define go_down 1
#define go_back 2
#define complete 3

#define unsafe false
#define safe true

using namespace std;

static int fsm_now;

static ros::Publisher command_pub;
static GridMap::Ptr grid_map_;
static double fov_x, fov_y;
static Eigen::Vector3d hover_pt_origin;
static Eigen::Vector3d hover_pt_upper;
static Eigen::Vector3d hover_pt_lower;
static Eigen::Vector3d hover_pt_setpoint;
static Eigen::Quaterniond hover_q_origin;
static Eigen::Quaterniond hover_q_setpoint;




static nav_msgs::Odometry uav_odom;
static bool got_odom = false;
void odometrycallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uav_odom = *msg;
    got_odom = true;
}

vector<Eigen::Vector3d> pos_to_be_check_upper()
{
    int vertical_delta_no = 100;
    double diff_z = 10.0;
    double delta_z = diff_z / vertical_delta_no;;
    double delta_xy = (diff_z / tan(fov_y / 2.0)) * 2.0 / 4.0;
    

    // cout<<delta_xy<<endl;
    // cout<<delta_z <<endl;

    // create a 5 x 5 block to test whether it has occupancy
    Eigen::Vector3d current_pos(
        uav_odom.pose.pose.position.x,
        uav_odom.pose.pose.position.y,
        uav_odom.pose.pose.position.z
    );

    vector<Eigen::Vector3d> return_block;
    
    
    Eigen::Vector3d temp_pos;

    temp_pos.x() = current_pos.x() ;
    temp_pos.y() = current_pos.y() ;

    for(int k = 0; k < vertical_delta_no; k++)
    {
        
        temp_pos.z() = current_pos.z() + k * delta_z;                   
        return_block.emplace_back(temp_pos);           
    }



    // cout<<return_block.size()<<endl;

    return return_block;

}

vector<Eigen::Vector3d> pos_to_be_check_lower()
{
    int vertical_delta_no = 100;
    double diff_z = 4.0;
    double delta_z = diff_z / vertical_delta_no;
    double delta_xy = (diff_z / tan(fov_y / 2.0)) * 2.0 / 4.0;
    

    // cout<<delta_xy<<endl;
    // cout<<delta_z <<endl;

    // create a 5 x 5 block to test whether it has occupancy
    Eigen::Vector3d current_pos(
        uav_odom.pose.pose.position.x,
        uav_odom.pose.pose.position.y,
        uav_odom.pose.pose.position.z
    );

    vector<Eigen::Vector3d> return_block;
    
    
    Eigen::Vector3d temp_pos;

    temp_pos.x() = current_pos.x() ;
    temp_pos.y() = current_pos.y() ;

    for(int k = 0; k < vertical_delta_no; k++)
    {
        
        temp_pos.z() = current_pos.z() - k * delta_z;    
        // if(i == j)
        // {
        //     std::cout<<temp_pos<<std::endl;
        // }               
        return_block.emplace_back(temp_pos);           
    }


    // cout<<return_block.size()<<endl;

    return return_block;

}


int judge_up_or_down()
{
    vector<Eigen::Vector3d> up_pos = pos_to_be_check_upper();
    vector<Eigen::Vector3d> below_pos = pos_to_be_check_lower();

    int up_occupied = 0;
    int below_occupied = 0;

    for(auto what : up_pos)
    {

        if(grid_map_->getInflateOccupancy(what))
            up_occupied++;
    }

    for(auto what : below_pos)
    {
        if(grid_map_->getInflateOccupancy(what))
            below_occupied++;
    }

    using namespace std;

    cout<<below_occupied<<endl;
    cout<<up_occupied<<endl;

    if(below_occupied > up_occupied)
    {
        ROS_ERROR("go up");
        return 1;
    }
    else
    {
        ROS_ERROR("go down");
        return -1;
    }

}


Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy)
{
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    return Eigen::Quaterniond(
        q.w(),
        q.x(),
        q.y(),
        q.z()
    );

}

Eigen::Vector3d q2rpy(Eigen::Quaterniond q)
{
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    tf::Quaternion q_(qx, qy, qz, qw);
    tf::Matrix3x3 m(q_);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // Print Euler angles

    return Eigen::Vector3d(roll, pitch, yaw);        
}


static std_msgs::String uav_status;
void uav_status_callback(const std_msgs::String::ConstPtr &msg)
{
    uav_status = *msg;
}


vector<Eigen::Vector3d> pos_to_be_check()
{
    int diff_no = 100;
    double diff_z = 0.5;
    double delta_z = diff_z / diff_no;
    cout<<"here: "<<delta_z<<endl;
    double delta_xy = (diff_z / tan(fov_y / 2.0)) * 2.0 / 4.0;
    

    // cout<<delta_xy<<endl;
    // cout<<delta_z <<endl;

    // create a 5 x 5 block to test whether it has occupancy
    Eigen::Vector3d current_pos(
        uav_odom.pose.pose.position.x,
        uav_odom.pose.pose.position.y,
        uav_odom.pose.pose.position.z
    );

    vector<Eigen::Vector3d> return_block;
    
    
    Eigen::Vector3d temp_pos;
    // for(int i = -2; i <= 2; i++)
    // {
    //     for(int j = -2; j <= 2; j++)
    //     {
    temp_pos.x() = current_pos.x() ;//+ i * delta_xy;
    temp_pos.y() = current_pos.y() ;//+ i * delta_xy;

    for(int k = 0; k < diff_no; k++)
    {
        if(fsm_now == go_up)
        {
            temp_pos.z() = current_pos.z() + k * delta_z;
        }
        else if (fsm_now == go_down)
        {
            temp_pos.z() = current_pos.z() - k * delta_z;
        }
        else
        {
            temp_pos.z() = current_pos.z();
        }   

        return_block.emplace_back(temp_pos);           
    }

    //     }

    // }

    // cout<<return_block.size()<<endl;

    return return_block;
}


static bool all_safe;
static vector<Eigen::Vector3d> to_be_check_block;
void check_collision_callback(const ros::TimerEvent& e)
{

    
    to_be_check_block.clear();

    to_be_check_block = pos_to_be_check();

    bool danger = false;

    for(auto what : to_be_check_block)
    {
        if(grid_map_->getInflateOccupancy(what))
        {
            danger = true;   
            break;            
        }
    }
    
    if(danger)
    {
        
        if(fsm_now != complete)
        {
            ROS_WARN("GONNA HIT! SWITCH STATE!!!");
            fsm_now = fsm_now + 1;
        }
            
    }        
    else
    {
        // do nothing
    }





    // cout<<judge_up_or_down()<<endl;


        // ROS_INFO("JUST ANOTHER DAY IN COLLISION CHECK...");
}


static trajectory_msgs::MultiDOFJointTrajectoryPoint cmd_setpoint;
void set_command()
{

    // cout<<fsm_now<<endl;

    Eigen::Vector3d uav_pos(
        uav_odom.pose.pose.position.x,
        uav_odom.pose.pose.position.y,
        uav_odom.pose.pose.position.z
    );

    geometry_msgs::Transform trans;
    
    // set position
    if(hover_pt_setpoint.z() + 0.2 > hover_pt_upper.z())
    {
        ROS_INFO("go_up to go_down!");
        fsm_now = go_down;
    }

    if(hover_pt_setpoint.z() - 0.2 < hover_pt_lower.z())
    {
        ROS_INFO("go_down to go_back!");
        fsm_now = go_back;
    }

    if(fsm_now == go_back )
    {
        if( abs((uav_pos.z() - hover_pt_origin.z())) < 0.1)
        {
            // cout<<(uav_pos.z() - hover_pt_origin.z())<<endl;
            ROS_INFO("go_back to complete!");
            fsm_now = complete;
        }
            
    }

    
    

    if(fsm_now == go_up)
    {
        hover_pt_setpoint.z() = hover_pt_setpoint.z() + 0.002;
    }
    else if(fsm_now == go_down)
    {
        hover_pt_setpoint.z() = hover_pt_setpoint.z() - 0.002;
    }
    else if(fsm_now == go_back)
    {
        hover_pt_setpoint.z() = hover_pt_setpoint.z() + 0.002;
    }    
    else
    {
        hover_pt_setpoint.z() = hover_pt_origin.z();
    }

    
    
    // set attitude
    double yaw_turning = q2rpy(hover_q_setpoint).z();

    if(fsm_now != complete)
    {
        yaw_turning = yaw_turning - 2.5 / 360.0 * 2 * M_PI;

    }
    else
    {
        // do nothing
    }


    hover_q_setpoint = rpy2q(
        Eigen::Vector3d(
            0.0,
            0.0,
            yaw_turning
        )
    );
    

    // cout<<yaw_turning<<endl;

    



    if(fsm_now == complete)
    {
        // cout<<hover_pt_setpoint<<endl<<endl;
    }
    // pass toe trans, velocity, and accleration

    trans.translation.x = hover_pt_setpoint.x();
    trans.translation.y = hover_pt_setpoint.y();
    trans.translation.z = hover_pt_setpoint.z();
    
    trans.rotation.w = hover_q_setpoint.w();
    trans.rotation.x = hover_q_setpoint.x();
    trans.rotation.y = hover_q_setpoint.y();
    trans.rotation.z = hover_q_setpoint.z();    

    // cout<<trans.rotation.z<<endl;

    geometry_msgs::Twist velocity;
    velocity.linear.x = 1.0;
    velocity.linear.y = 1.0;
    velocity.linear.z = 1.0;
    velocity.angular.x = 1.0;
    velocity.angular.y = 0.5;
    velocity.angular.z = 0.5;

    geometry_msgs::Twist acceleration;
    acceleration.linear.x = 1.0;
    acceleration.linear.y = 1.0;
    acceleration.linear.z = 1.0;
    acceleration.angular.x = 0.5;
    acceleration.angular.y = 0.5;
    acceleration.angular.z = 0.5;

    cmd_setpoint.transforms.clear();
    cmd_setpoint.velocities.clear();
    cmd_setpoint.accelerations.clear();
    // cmd_setpoint.time_from_start.
    
    cmd_setpoint.transforms.push_back(trans);
    cmd_setpoint.velocities.push_back(velocity);
    cmd_setpoint.accelerations.push_back(acceleration);
    cmd_setpoint.time_from_start = ros::Duration(2.5);

}


void planner_command_callback(const ros::TimerEvent& e)
{
    set_command();
    command_pub.publish(cmd_setpoint);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_fsm_eg");
    ros::NodeHandle nh("~");            

    ros::Subscriber uav_odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/red/odometry", 1, odometrycallback);

    ros::Subscriber uav_status_sub = nh.subscribe<std_msgs::String>
        ("/red/carrot/status", 1 , uav_status_callback);

    command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/red/position_hold/trajectory", 1);
    
    nh.getParam("fov_x", fov_x);
    nh.getParam("fov_y", fov_y);


    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ros::Timer check_collision = nh.createTimer(
        ros::Duration(0.01),
        &check_collision_callback
    );

    ros::Timer maneuver_command = nh.createTimer(
        ros::Duration(0.01),
        &planner_command_callback
    );
    
    
    //pre-flight check
    while(ros::ok())
    {
        if(got_odom)
        {
            hover_pt_origin.x() = uav_odom.pose.pose.position.x;
            hover_pt_origin.y() = uav_odom.pose.pose.position.y;
            hover_pt_origin.z() = uav_odom.pose.pose.position.z;

            hover_pt_setpoint = hover_pt_origin;

            hover_pt_upper = hover_pt_origin;
            hover_pt_upper.z() = hover_pt_upper.z() + 2.5;

            hover_pt_lower = hover_pt_origin;
            hover_pt_lower.z() = hover_pt_lower.z() - 2.5;

            hover_q_origin.w() = uav_odom.pose.pose.orientation.w;
            hover_q_origin.x() = uav_odom.pose.pose.orientation.x;
            hover_q_origin.y() = uav_odom.pose.pose.orientation.y;
            hover_q_origin.z() = uav_odom.pose.pose.orientation.z;

            hover_q_setpoint = hover_q_origin;

            break;
        }
            
        ros::spinOnce();             
    }

    ROS_INFO("NOW GOOD TO GO");
    fsm_now = go_up;


    ros::spin();

    return 0;
}