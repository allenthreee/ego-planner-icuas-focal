#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>

#include <plan_env/grid_map.h>

// #include "./poi.h"

#include <icuas23_competition/poi.h>

#include <tf/transform_datatypes.h>


// #include <ego_planner/DataDisp.h>

#define HOVER_IDLE 0
#define HOVER_FOR_EGO 1
#define HOVER_FOR_FUEL 2
#define EGO 3
#define EGO_GOT_STUCK 6
#define FUEL 4
#define TERMINATE 5

#define vexp_go_up 100
#define vexp_go_down 101
#define vexp_go_back 102
#define vexp_complete 103

class a_fsm
{
private:
    /* data */
    int fsm_now = HOVER_IDLE;

    ros::NodeHandle nh;
    ros::Subscriber uav_odom_sub, 
        uav_status_sub, 
        ego_command_sub,
        fuel_command_sub,
        fuel_terminate_sub,
        tsped_poi_sub,
        stuck_sub, 
        poi_sub;

    ros::Publisher local_waypoints_toego_pub,
        local_poi_tofuel_pub,
        controller_pub,
        fuel_trigger_pub;

    ros::Timer mainspin_timer;

    GridMap::Ptr grid_map_;
    
                
    bool got_odom = false;
    nav_msgs::Odometry uav_odom;
    void uav_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

    std_msgs::String uav_status;
    void uav_status_callback(const std_msgs::String::ConstPtr &msg);
    
    
    trajectory_msgs::MultiDOFJointTrajectoryPoint 
        hover_command, ego_command, fuel_command, final_command;
    void ego_command_callback(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg
    );
    void fuel_command_callback(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg
    );

    void fuel_terminate_callback(const std_msgs::Bool::ConstPtr &msg);
    bool fuel_terminate = false;

    void poi_callback(const icuas23_competition::poi::ConstPtr & msg);

    std_msgs::Bool ego_got_stuck;
    bool stuck_or_not = false;
    int got_stuck_counter = 0;
    void got_stuck_callback(const std_msgs::Bool::ConstPtr& msg);
    std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> traj_legacy;
    int legacy_indicator;

    //concerning point of interests
    int total_cities_no;
    nav_msgs::Path TSPed_POT;
    bool got_TSPed_POI = false;
    void tsped_poi_callback(
        const nav_msgs::Path::ConstPtr &msg
    );


    bool mission_init = false;
    double last_request;
    int we_now_about2_ego_or_fuel = EGO;

    bool got_command = false;

    double yaw_turning = 0;

    int judge_up_or_down();


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

    //concerning hover
    bool set_next_city;

    double fov_x, fov_y;
    


    //concerning ego
    bool ego_flag = true;
    void pub_local_wp();
    void pub_local_wp_stuck();
    bool infer_ego_arrive();
    bool publishwp = false;
    bool arriveornot = false;
    double localwp_last_request;
    int city_indicator = 0;
    nav_msgs::Path current_pos_target;

    //concerning fuel
    // std_msgs::Bool 
    bool got_fuel_command = false;//continuously published by fuel
    // bool got_ego_command = false;

    void mainspin_callback(
        const ros::TimerEvent &e
    );

    void mainspinserver();

    vector<Eigen::Vector3d> pos_to_be_check_upper();
    vector<Eigen::Vector3d> pos_to_be_check_lower();

    bool set_go_up_or_down = false;
    int go_up_or_down = 0;

    Eigen::Vector3d hover_pt_origin_for_stuck;
    Eigen::Vector3d hover_pt_origin_for_stuck_setpt;
    
    int fsm_vexp = vexp_go_up;
    Eigen::Vector3d hover_pt_setpoint, 
    hover_pt_upper,
    hover_pt_lower,
    hover_pt_origin;
    Eigen::Quaterniond hover_q_origin;
    Eigen::Quaterniond hover_q_setpoint;
    void set_vexp();
    vector<Eigen::Vector3d> pos_to_be_check();
    trajectory_msgs::MultiDOFJointTrajectoryPoint vexp_set_command();
    void check_collision_callback();
    vector<Eigen::Vector3d> to_be_check_block;

    double trigger_ego_stuck_last_request = 0;



public:
    a_fsm(ros::NodeHandle& _nh);
    ~a_fsm();
};
