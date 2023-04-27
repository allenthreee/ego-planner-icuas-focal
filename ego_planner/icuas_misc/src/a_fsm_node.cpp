#include "../include/a_fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_fsm_node");
    ros::NodeHandle nh("~");

    a_fsm a_fsm_node(nh);
    ros::spin();
    
    return 0;
}