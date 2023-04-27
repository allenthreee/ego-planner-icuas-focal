#include "../include/a_fsm.h"

a_fsm::a_fsm(ros::NodeHandle& _nh)
: nh(_nh)
{
    //subscriber

    uav_odom_sub = nh.subscribe<nav_msgs::Odometry>
        ("/red/odometry", 1, &a_fsm::uav_odom_callback, this);
    uav_status_sub = nh.subscribe<std_msgs::String>
        ("/red/carrot/status", 1, &a_fsm::uav_status_callback, this);

    ego_command_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/ego/red/position_hold/trajectory", 1, &a_fsm::ego_command_callback, this);
    
    fuel_command_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/fuel/red/position_hold/trajectory", 1, &a_fsm::fuel_command_callback, this);
    
    fuel_terminate_sub = nh.subscribe<std_msgs::Bool>
        ("/fuel_finish", 1, &a_fsm::fuel_terminate_callback, this);
    

    tsped_poi_sub = nh.subscribe<nav_msgs::Path>
        ("/tsped_pois", 1, &a_fsm::tsped_poi_callback, this);

    stuck_sub = nh.subscribe<std_msgs::Bool>
        ("/got_stuck", 1, &a_fsm::got_stuck_callback, this);


    // poi_sub = nh.subscribe<icuas23_competition::poi>
    //     ("/red/poi", 1, &a_fsm::poi_callback, this);

    //publisher
    local_waypoints_toego_pub = nh.advertise<nav_msgs::Path>
        ("/waypoint_generator/waypoints", 1);
    local_poi_tofuel_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/poi_topic", 1);// = nh.advertise<...>("", 1);


    // geometry_msgs::PoseStamped>("/poi_topic"

    fuel_trigger_pub = nh.advertise<std_msgs::Bool>
        ("/fuel_start", 1);

    controller_pub = nh.advertise
        <trajectory_msgs::MultiDOFJointTrajectoryPoint>
        ("/red/position_hold/trajectory", 1);

    nh.getParam("fov_x", fov_x);
    nh.getParam("fov_y", fov_y);
    

    
    //timer
    mainspin_timer = nh.createTimer(
        ros::Duration(0.01),
        &a_fsm::mainspin_callback,
        this
    );

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);


    ros::Duration(1.0).sleep();

    while(ros::ok())
    {
        ros::spinOnce();
        if(got_odom && got_TSPed_POI)
        {
            break;
            ROS_INFO("MISSION START!");
        }
    }

}

a_fsm::~a_fsm(){}



void a_fsm::poi_callback(const icuas23_competition::poi::ConstPtr & msg)
{
    std::cout<<"hi..."<<std::endl;    
}

void a_fsm::uav_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    using namespace std;
    // std::cout<<"hi"<<std::endl;

    uav_odom = *msg;
    got_odom = true;

}

void a_fsm::uav_status_callback(const std_msgs::String::ConstPtr &msg)
{
    uav_status = *msg;
}

void a_fsm::ego_command_callback(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg
)
{
    // std::cout<<"ego_command"<<std::endl<<std::endl;;
    ego_command = *msg;
    got_command = true;

}

void a_fsm::fuel_command_callback(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg
)
{
    std::cout<<"hi here receive fuel command"<<std::endl;
    fuel_command = *msg;
    got_fuel_command = true;
    got_command = true;

}

void a_fsm::tsped_poi_callback(
    const nav_msgs::Path::ConstPtr &msgs
)
{
    // std::cout<<"hi"<<std::endl;
    if(!got_TSPed_POI)
    {
        TSPed_POT = *msgs;
        got_TSPed_POI = true;
        total_cities_no = TSPed_POT.poses.size();
    }
    //only listen to this topic once

    // std::cout<<"total cities number: "<<total_cities_no<<std::endl;
}

void a_fsm::fuel_terminate_callback(
    const std_msgs::Bool::ConstPtr &msg
)
{
    fuel_terminate = msg->data;
}

void a_fsm::mainspin_callback(const ros::TimerEvent&e)
{
    // ROS_INFO("MAINSERVER CALLBACK");
    // std::cout<<uav_status.data<<std::endl;
    if(
        got_odom && 
        got_TSPed_POI 
    )
    {
        using namespace std;
        mainspinserver();
        
        if( (got_command && fsm_now != TERMINATE) || fsm_now == EGO_GOT_STUCK)
        {
            controller_pub.publish(final_command);            
            got_command = false;
        }        
    }                        
}


void a_fsm::mainspinserver()
{
    switch (fsm_now)
    {
    case HOVER_IDLE:

        ROS_INFO("HOVER_IDLE");
        if(!mission_init)
        {
            last_request = ros::Time::now().toSec();
            mission_init = true;
            fsm_now = HOVER_FOR_EGO;
        }
        
        break;

    case HOVER_FOR_EGO:

        if(got_command)
            final_command = ego_command;

        ROS_INFO("HOVER_FOR_EGO");
        if(
            (ros::Time::now().toSec() - last_request)
            >
            ros::Duration(4.0).toSec()
        )
        {
            fsm_now = EGO;
            publishwp = true;
            
            last_request = ros::Time::now().toSec();
        }
        
        break;
    
    case EGO:
        /* code */
        ROS_INFO("EGO");                 

        pub_local_wp();//
        if(fsm_now == TERMINATE)
            break;

        if(got_command)
            final_command = ego_command;

        if(infer_ego_arrive())
        {
            fsm_now = HOVER_FOR_FUEL;
            yaw_turning = q2rpy(
                Eigen::Quaterniond(
                    uav_odom.pose.pose.orientation.w,
                    uav_odom.pose.pose.orientation.x,
                    uav_odom.pose.pose.orientation.y,
                    uav_odom.pose.pose.orientation.z
                )
            )(2);
            last_request = ros::Time::now().toSec();
        }

        if(stuck_or_not == true)
        {
            ROS_WARN("GOT STUCK LAHH");
            fsm_now = EGO_GOT_STUCK;
            set_go_up_or_down = true;
            hover_pt_origin_for_stuck = Eigen::Vector3d(
                uav_odom.pose.pose.position.x,
                uav_odom.pose.pose.position.y,
                uav_odom.pose.pose.position.z
            );
            hover_pt_origin_for_stuck_setpt = hover_pt_origin_for_stuck;
            break;

        }

        break;

    case EGO_GOT_STUCK:
        // ROS_INFO("EGO_GOT_STUCK");
        if(set_go_up_or_down)
        {
            go_up_or_down = judge_up_or_down();
            trigger_ego_stuck_last_request = ros::Time::now().toSec();
            set_go_up_or_down = false;
        }

        if(go_up_or_down == 1)
        {
            hover_pt_origin_for_stuck_setpt.z() = hover_pt_origin_for_stuck_setpt.z() + 0.002;
            final_command.transforms[0].translation.z = hover_pt_origin_for_stuck_setpt.z();
        }
        else if(go_up_or_down == -1)
        {
            hover_pt_origin_for_stuck_setpt.z() = hover_pt_origin_for_stuck_setpt.z() - 0.002;
            final_command.transforms[0].translation.z = hover_pt_origin_for_stuck_setpt.z();
        }

        pub_local_wp_stuck();

        if(stuck_or_not == false)
        {
            fsm_now = EGO;
        }

        break;

     case HOVER_FOR_FUEL:
        /* code */
        ROS_INFO("HOVER_FOR_FUEL");
        if(            
            // got_fuel_command //bool init = false
            // remember to change to false
            (ros::Time::now().toSec() - last_request)
            > 
            ros::Duration(2.0).toSec()
        )
        {
            fsm_now = FUEL;
            set_vexp();
            last_request = ros::Time::now().toSec();
        }

        break;

    case FUEL:
        /* code */
        // ROS_INFO("VEXP");

        check_collision_callback();
        final_command = vexp_set_command();

        if(fsm_vexp == vexp_complete || 
            ros::Time::now().toSec() - last_request > ros::Duration(20.0).toSec())
        {
            fsm_now = HOVER_FOR_EGO;
            last_request = ros::Time::now().toSec();
        }

        break;

    case TERMINATE:
        /* code */
        ROS_INFO("TERMINATE");
        ros::shutdown();
        // final_command;
        break;
    
    default:
        break;
    }


}

void a_fsm::set_vexp()
{
    hover_pt_origin.x() = uav_odom.pose.pose.position.x;
    hover_pt_origin.y() = uav_odom.pose.pose.position.y;
    hover_pt_origin.z() = uav_odom.pose.pose.position.z;

    hover_pt_setpoint = hover_pt_origin;

    hover_pt_upper = hover_pt_origin;
    hover_pt_upper.z() = hover_pt_upper.z() + 1.8;

    hover_pt_lower = hover_pt_origin;
    hover_pt_lower.z() = hover_pt_lower.z() - 1.8;

    if(hover_pt_lower.z() < 0)
        hover_pt_lower.z() = 0.5;

    hover_q_origin.w() = uav_odom.pose.pose.orientation.w;
    hover_q_origin.x() = uav_odom.pose.pose.orientation.x;
    hover_q_origin.y() = uav_odom.pose.pose.orientation.y;
    hover_q_origin.z() = uav_odom.pose.pose.orientation.z;

    hover_q_setpoint = hover_q_origin;

    fsm_vexp = vexp_go_up;
    
}

vector<Eigen::Vector3d> a_fsm::pos_to_be_check()
{
    double diff_z = 0.5;
    double delta_z = diff_z / 5.0;
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
    for(int i = -2; i <= 2; i++)
    {
        for(int j = -2; j <= 2; j++)
        {
            temp_pos.x() = current_pos.x() + i * delta_xy;
            temp_pos.y() = current_pos.y() + i * delta_xy;

            for(int k = 0; k < 5; k++)
            {
                if(fsm_vexp == vexp_go_up)
                {
                    temp_pos.z() = current_pos.z() + k * delta_z;
                }
                else if (fsm_vexp == vexp_go_down)
                {
                    temp_pos.z() = current_pos.z() - k * delta_z;
                }
                else
                {
                    temp_pos.z() = current_pos.z();
                }   

                return_block.emplace_back(temp_pos);           
            }

        }

    }

    return return_block;
}

void a_fsm::check_collision_callback()
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
        ROS_WARN("GONNA HIT! SWITCH STATE!!!");
        if(fsm_vexp == vexp_go_up)
            fsm_vexp == vexp_go_down;
        else if(fsm_vexp == vexp_go_down)
            fsm_vexp == vexp_complete;
        if(fsm_vexp != vexp_complete)
            fsm_vexp = fsm_vexp + 1;
    }        
    else
    {
        // do nothing
    }





    // cout<<judge_up_or_down()<<endl;


        // ROS_INFO("JUST ANOTHER DAY IN COLLISION CHECK...");
}



trajectory_msgs::MultiDOFJointTrajectoryPoint a_fsm::vexp_set_command()
{
    trajectory_msgs::MultiDOFJointTrajectoryPoint cmd_setpoint;
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
        fsm_vexp = vexp_go_down;
    }

    if(hover_pt_setpoint.z() - 0.2 < hover_pt_lower.z())
    {
        ROS_INFO("go_down to go_back!");
        fsm_vexp = vexp_go_back;
    }

    if(fsm_vexp == vexp_go_back )
    {
        if( abs((uav_pos.z() - hover_pt_origin.z())) < 0.1)
        {
            // cout<<(uav_pos.z() - hover_pt_origin.z())<<endl;
            ROS_INFO("go_back to complete!");
            fsm_vexp = vexp_complete;
        }
            
    }
        

    if(fsm_vexp == vexp_go_up)
    {
        hover_pt_setpoint.z() = hover_pt_setpoint.z() + 0.002;
    }
    else if(fsm_vexp == vexp_go_down)
    {
        hover_pt_setpoint.z() = hover_pt_setpoint.z() - 0.002;
    }
    else if(fsm_vexp == vexp_go_back)
    {
        hover_pt_setpoint.z() = hover_pt_setpoint.z() + 0.002;
    }    
    else
    {
        hover_pt_setpoint.z() = hover_pt_origin.z();
    }

    
    
    // set attitude
    double yaw_turning = q2rpy(hover_q_setpoint).z();

    if(fsm_vexp != vexp_complete)
    {
        yaw_turning = yaw_turning - 0.4 / 360.0 * 2 * M_PI;
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

    if(fsm_vexp == vexp_complete)
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

    ego_command.transforms[0].translation.x = hover_pt_setpoint.x();
    ego_command.transforms[0].translation.y = hover_pt_setpoint.y();
    ego_command.transforms[0].translation.z = hover_pt_setpoint.z();

    ego_command.transforms[0].rotation.w = hover_q_setpoint.w();
    ego_command.transforms[0].rotation.x = hover_q_setpoint.x();
    ego_command.transforms[0].rotation.y = hover_q_setpoint.y();
    ego_command.transforms[0].rotation.z = hover_q_setpoint.z();


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

    return ego_command;
}



void a_fsm::pub_local_wp()
{
    std::cout<<"pub_local_wp"<<std::endl;

    if(publishwp)
    {
        using namespace std;
        cout<<"city_indicator: "<<city_indicator<<endl;
        cout<<"total_cities_no: "<<total_cities_no<<endl;
        if(city_indicator >= total_cities_no)
        {
            fsm_now = TERMINATE;
            return;
        }
        localwp_last_request = ros::Time::now().toSec();
        publishwp = false;
        current_pos_target.poses.clear();
        current_pos_target.poses.emplace_back(TSPed_POT.poses[city_indicator]);

        city_indicator++;
    }

    if(
        (ros::Time::now().toSec() - localwp_last_request)
        <
        ros::Duration(4.0).toSec()
    )
    {
        std::cout<<"hi pub pose to ego"<<std::endl;
        local_waypoints_toego_pub.publish(current_pos_target);        
    }

}

void a_fsm::pub_local_wp_stuck()
{
    if(ros::Time::now().toSec() - trigger_ego_stuck_last_request > ros::Duration(2.0).toSec())
    {
        local_waypoints_toego_pub.publish(current_pos_target);
        trigger_ego_stuck_last_request = ros::Time::now().toSec();
    }
}

bool a_fsm::infer_ego_arrive()
{
    Eigen::Vector3d current_pos;
    current_pos << 
        uav_odom.pose.pose.position.x,
        uav_odom.pose.pose.position.y,
        uav_odom.pose.pose.position.z;
    
    Eigen::Vector3d local_des_pos;
    local_des_pos << 
        current_pos_target.poses[0].pose.position.x,
        current_pos_target.poses[0].pose.position.y,
        current_pos_target.poses[0].pose.position.z;

        std::cout<<(current_pos - local_des_pos).norm()<<std::endl;

    if((current_pos - local_des_pos).norm() < 0.4)
        return true;
    else
        return false;
        
}


void a_fsm::got_stuck_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        got_stuck_counter++;

    }
    else
    {
        got_stuck_counter = 0;
        stuck_or_not = false;
    }

    if(got_stuck_counter > 50)
    {
        stuck_or_not = true;
    }
}


vector<Eigen::Vector3d> a_fsm::pos_to_be_check_upper()
{
    int vertical_delta_no = 100;
    double diff_z = 4.0;
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
    for(int i = -2; i <= 2; i++)
    {
        for(int j = -2; j <= 2; j++)
        {
            temp_pos.x() = current_pos.x() + i * delta_xy;
            temp_pos.y() = current_pos.y() + i * delta_xy;

            for(int k = 0; k < vertical_delta_no; k++)
            {
                
                temp_pos.z() = current_pos.z() + k * delta_z;                   
                return_block.emplace_back(temp_pos);           
            }

        }

    }

    // cout<<return_block.size()<<endl;

    return return_block;

}

vector<Eigen::Vector3d> a_fsm::pos_to_be_check_lower()
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
    for(int i = -2; i <= 2; i++)
    {
        for(int j = -2; j <= 2; j++)
        {
            temp_pos.x() = current_pos.x() + i * delta_xy;
            temp_pos.y() = current_pos.y() + i * delta_xy;

            for(int k = 0; k < vertical_delta_no; k++)
            {
                
                temp_pos.z() = current_pos.z() - k * delta_z;    
                if(i == j)
                {
                    std::cout<<temp_pos<<std::endl;
                }               
                return_block.emplace_back(temp_pos);           
            }

        }

    }

    // cout<<return_block.size()<<endl;

    return return_block;

}


int a_fsm::judge_up_or_down()
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