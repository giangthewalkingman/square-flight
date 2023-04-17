#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include<tf/transform_datatypes.h>
//#include<mavros_msgs

class MultiDOFControl {
    public:
    MultiDOFControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool manual_input);
    ~MultiDOFControl();
    //void setGlobalTrajectory();
    private:
    const double PI = 3.141592653589793238463; // PI

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    //ros::Subscriber local_p_sub_;
    //ros::Subscriber gps_position_sub_; // current gps position subscriber
    ros::Subscriber odom_sub_; // odometry subscriber
    ros::Subscriber state_sub_;

    ros::Publisher odom_error_pub_; //publish odom error before arm
    ros::Publisher traj_pub_; //publish trajectory
    ros::Publisher setpoint_p_pub_;
    ros::Publisher opt_point_pub_;
    ros::Publisher mavros_pub_;
    ros::Publisher command_pub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    mavros_msgs::SetMode offboard_setmode_; //check mode flight of uav


    //void poseCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg); //call back the current position

    //trajectory_msgs::MultiDOFJointTrajectoryPoint local_DOFsetpoint_[10];
    //trajectory_msgs::MultiDOFJointTrajectoryPoint target_pose_;
    //geometry_msgs::Point global_setpoint_[64];

    mavros_msgs::State current_state_; //drone current state
    mavros_msgs::SetMode flight_mode_; // use to set custom or default flight mode (e.g., OFFBOARD, LAND, ...)
    mavros_msgs::PositionTarget pos_target_;
    ros::Time operation_time_1_, operation_time_2_;
    bool simulation_mode_enable_;
    bool odom_error_;
    bool odom_received_ = false; // check received odometry or not
    //bool manual_input_; //input manually by commander

    geometry_msgs::PoseStamped current_position_;

    geometry_msgs::Transform tf_[10];
    geometry_msgs::Twist vel_, acc_;
    geometry_msgs::PoseStamped target_enu_pose_;
    trajectory_msgs::MultiDOFJointTrajectory traj_msg_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint opt_points[10]; //an array of optimization points
    trajectory_msgs::MultiDOFJointTrajectoryPoint control_point_; //the control point
    //void setGlobalTrajectory();
    nav_msgs::Odometry current_odom_;

    double vel_desired_;
    double vel_planning_desired_;
    double geo_error_;
    double hover_time_;
    double land_error_;
    geometry_msgs::Vector3 components_vel_;
    bool checkPositionError(double geo_error, geometry_msgs::PoseStamped target_position);
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);


    void setMultiDOFPoints();
    void multiDOFFlight();
    void mapCheckingFlight(double hz);
    void setOffboardStream(double hz, trajectory_msgs::MultiDOFJointTrajectoryPoint first_target);
    void distanceBetween(geometry_msgs::PoseStamped current_position, geometry_msgs::PoseStamped target_position);
    geometry_msgs::PoseStamped targetTransfer(double x, double y, double z);
    //double z_takeoff_;
    void waitForPredicate(double hz); // wait for connect, GPS received, ...
    void commander();
    void setOffboardStream(double hz, geometry_msgs::PoseStamped first_target);
    void waitForArmAndOffboard(double hz);
    void takeOff(geometry_msgs::PoseStamped setpoint, double hover_time);
    void hovering(geometry_msgs::PoseStamped setpoint, double hover_time);
    void landing(geometry_msgs::PoseStamped setpoint);
    // geometry_msgs::PoseStamped poseStampedFromVector3(geometry_msgs::Vector3 current) {
    //     geometry_msgs::PoseStamped target;
    //     target.pose.position.x = current.x;
    // }

    geometry_msgs::Vector3 velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
    template <class T>
    inline T sqr(T x) // calculate square of x
	{
        return x*x;
    }; 



};

void setGlobalTrajectory();

geometry_msgs::Point global_setpoint_[64];
double z_takeoff_;

#endif