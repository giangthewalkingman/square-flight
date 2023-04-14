#include <square_flight/offboard.h>

int main(int argc, char **argv) 
{
    setGlobalTrajectory();
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    MultiDOFControl *offboard = new MultiDOFControl(nh, nh_private, true);
    ros::spin();
    
    return 0;
}