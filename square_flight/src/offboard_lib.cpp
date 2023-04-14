#include "square_flight/offboard.h"

MultiDOFControl::MultiDOFControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool manual_input): nh_(nh),
                                                                                                nh_private_(nh_private),
                                                                                                simulation_mode_enable_(false) {
    //local_p_sub_ = nh_.subscribe("/mavros/local_position/pose",10, &MultiDOFControl::poseCallback, this);

    //traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("mavros/setpoint_raw/multidof", 10);
    setpoint_p_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    //command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
    //opt_point_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/mavros/setpoint_raw/local", 10);
    //opt_point_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/mavros/setpoint_position/cmd_vel", 10);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &MultiDOFControl::odomCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 10, &MultiDOFControl::stateCallback, this);
    odom_error_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_error", 1, true);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    nh_private_.getParam("/offb_node/odom_error", odom_error_);
    nh_private_.getParam("/offb_node/target_error", geo_error_);
    nh_private_.getParam("/offb_node/land_error", land_error_);
    nh_private_.getParam("/offb_node/desired_velocity", vel_desired_);
    nh_private_.getParam("/offb_node/hover_time", hover_time_);
    nh_private_.param<bool>("/offb_node/simulation_mode_enable", simulation_mode_enable_, simulation_mode_enable_);
    waitForPredicate(10.0);
    if(manual_input == true) {
        commander();
    }
}

MultiDOFControl::~MultiDOFControl() {
    //Destructive function
}


void MultiDOFControl::waitForPredicate(double hz) {
    std::cout<<"Desired velocity is "<<vel_desired_<<std::endl;
    std::cout<<"Target error is "<<geo_error_<<std::endl;
    //std::cout<<"Land erorr is "<<land_error_<<std::endl;
    ros::Rate rate(hz);

    std::printf("\n[ INFO] Waiting for FCU connection \n");
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] FCU connected \n");
    if (simulation_mode_enable_) {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set true\n");
        std::printf("          OFFBOARD node will automatic ARM and set OFFBOARD mode\n");
        std::printf("          Continue if run a simulation OR SHUTDOWN node if run in drone\n");
        std::printf("          Set parameter 'simulation_mode_enable' to false or not set (default = false)\n");
        std::printf("          and relaunch node for running in drone\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=false\n");
    }
    else {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set false or not set (default = false)\n");
        std::printf("          OFFBOARD node will wait for ARM and set OFFBOARD mode from RC controller\n");
        std::printf("          Continue if run in drone OR shutdown node if run a simulation\n");
        std::printf("          Set parameter 'simulation_mode_enable' to true and relaunch node for simulation\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=true\n");
    }
    operation_time_1_ = ros::Time::now();
}

/* send a few setpoints before publish
   input: ros rate in hertz (at least 2Hz) and first setpoint */
void MultiDOFControl::setOffboardStream(double hz, geometry_msgs::PoseStamped first_target) {
    ros::Rate rate(hz);
    std::printf("[ INFO] Setting OFFBOARD stream \n");
    for (int i = 50; ros::ok() && i > 0; --i) {
        target_enu_pose_ = first_target;
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_p_pub_.publish(target_enu_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("\n[ INFO] OFFBOARD stream is set\n");
}

/* wait for ARM and OFFBOARD mode switch (in SITL case or HITL/Practical case)
   input: ros rate in hertz, at least 2Hz */
void MultiDOFControl::waitForArmAndOffboard(double hz) {
    ros::Rate rate(hz);
    if (simulation_mode_enable_) {
        std::printf("\n[ INFO] Ready to takeoff\n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD")) {
            mavros_msgs::CommandBool arm_amd;
            arm_amd.request.value = true;
             if (arming_client_.call(arm_amd) && arm_amd.response.success) {
                ROS_INFO_ONCE("Vehicle armed");
            }
            else {
                ROS_INFO_ONCE("Arming failed");
            }

            // mavros_msgs::SetMode offboard_setmode_;
            offboard_setmode_.request.base_mode = 0;
            offboard_setmode_.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(offboard_setmode_) && offboard_setmode_.response.mode_sent) {
                ROS_INFO_ONCE("OFFBOARD enabled");
            }
            else {
                ROS_INFO_ONCE("Failed to set OFFBOARD");
            }
            ros::spinOnce();
            rate.sleep();
        }
        //DuyNguyen
        if (odom_error_) {
            odom_error_pub_.publish(current_odom_);
        }
    }
    else {
        std::printf("\n[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC\n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD")) {
            ros::spinOnce();
            rate.sleep();
        }
        if (odom_error_) {
            odom_error_pub_.publish(current_odom_);
        }
    }
}

void MultiDOFControl::stateCallback(const mavros_msgs::State::ConstPtr &msg) {
     //std::cout << "In stateCallBack()" << std::endl;
    current_state_ = *msg;
}

// void MultiDOFControl::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
//     // std::cout << "In poseCallBack" << std::endl;
//     current_position_ = *msg;
// }

void MultiDOFControl::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odom_ = *msg;
    odom_received_ = true;
}

void MultiDOFControl::commander() {
    // ros::Rate rate(10);
    // int i = 0;
    // while(ros::ok()) {
    //     std::cout << i << std::endl;
    //     i++;
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    std::cout << "Input the height: ";
    std::cin >> z_takeoff_;
    std::cout << "Press (1) to start flying, (0) to cancel: ";
    char mode;
    std::cin >> mode;
    if(mode == '1') {
        ::setGlobalTrajectory();
        setOffboardStream(10, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
        waitForArmAndOffboard(10);
        setMultiDOFPoints();
        takeOff(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_), hover_time_);
        //mapCheckingFlight(10);
        multiDOFFlight();
        //landing()
    }
    else {
        commander();
    }
}



void setGlobalTrajectory() {
    //set x,y,z for global setpoints to make a square trajectory
    for(int i = 0; i < 64; i++) {
        global_setpoint_[i].z = z_takeoff_; //set z
        if(0 <= i && i < 16) {
            global_setpoint_[i].x = i; //increase x
            global_setpoint_[i].y = 0;
        }
        if(16 <= i && i< 32) {
            global_setpoint_[i].x = 16;
            global_setpoint_[i].y = i - 16; //increase y
        }
        if(32 <= i && i < 48) {
            global_setpoint_[i].x = 48 -i; //decrease x
            global_setpoint_[i].y = 16;
        }
        if(48 <= i && i < 64) {
            global_setpoint_[i].x = 0;
            global_setpoint_[i].y = 64 - i; //decrease y
        }
    }
    for(int i = 0; i < 64; i++) {
        std::cout << "Setpoint (" <<i <<"): " << std::endl;
        std::cout << global_setpoint_[i].x <<", " <<global_setpoint_[i].y << ", "<<global_setpoint_[i].z<<std::endl;
    }
    // global_setpoint_[5].x = 7;
    // global_setpoint_[5].y = 5;
    // global_setpoint_[5].z = 10;
}

void MultiDOFControl::setMultiDOFPoints() {
    for(int i = 0; i < 10; i ++){
        //set rotation
        tf_[i].rotation.x = 0;
        tf_[i].rotation.y = 0;
        tf_[i].rotation.z = 0;
        tf_[i].rotation.w = 1;
        //set velocity
        vel_.angular.x = 0;
        vel_.angular.y = 0;
        vel_.angular.z = 0;
        vel_.linear.x = vel_desired_;
        vel_.linear.y = vel_desired_;
        vel_.linear.z = vel_desired_;
        //set acceleration
        acc_.angular.x = 0;
        acc_.angular.y = 0;
        acc_.angular.z = 0;
        acc_.linear.x = 0;
        acc_.linear.y = 0;
        acc_.linear.z = 0;
    }
}

void MultiDOFControl::mapCheckingFlight(double hz) {
    std::cout << "Start map checking flight!" << std::endl;
    ros::Rate rate(hz);
    int i = 0;
    bool target_reached = false;
    bool final_target_reached = false; 
    //bool flag = true;
    if(i == 64) {
        final_target_reached = checkPositionError(geo_error_, targetTransfer(global_setpoint_[i].x, global_setpoint_[i].y, global_setpoint_[i].z));
        if(final_target_reached == false) {
            landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
            //final_target_reached = true;
        }
    }
    while(ros::ok() && !final_target_reached) {
        target_enu_pose_.pose.position = global_setpoint_[i];
        setpoint_p_pub_.publish(target_enu_pose_);
        target_reached = checkPositionError(geo_error_, targetTransfer(global_setpoint_[i].x, global_setpoint_[i].y, global_setpoint_[i].z));
        if(target_reached) {
            std::cout << "Reached target " << i << "! " << std::endl;
            i++;
        }
        if(i == 64) {
            //i = 0;
            target_enu_pose_.pose.position = global_setpoint_[0];
            setpoint_p_pub_.publish(target_enu_pose_);
            std::cout << "Reached final target" << std::endl;
            //landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
        }
        ros::spinOnce();
        rate.sleep();
    }
    landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
}

void MultiDOFControl::multiDOFFlight() {
    //geometry_msgs::PoseStamped xyz_reached;
    ros::Rate rate(10.0);
    //bool target_reached = checkPositionError(geo_error_, );
    bool first_target_reached = false;
    bool final_target_reached = false;
    bool target_reached = false;
    first_target_reached = checkPositionError(geo_error_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
    int i = 0;
    if(first_target_reached) {
        std:: cout << "Start multi DOF Flight." << std::endl;
        target_reached = true;
    }
    while(ros::ok() ) {
        if(0 <= i && i <= 54) {
            for(int j = 0; j < 10; j++) {
                tf_[j].translation.x = global_setpoint_[i+j].x;
                tf_[j].translation.y = global_setpoint_[i+j].y;
                tf_[j].translation.z = global_setpoint_[i+j].z;
                std::cout << "Local transform " << j << ": " << std::endl;
                std::cout << tf_[j].translation.x << ", " << tf_[j].translation.y << ", " << tf_[j].translation.z << std::endl;
                std::cout << "Global setpoint " << i+j << ": " << std::endl;
                //ROS_INFO_STREAM(global_setpoint_);
                std::cout << global_setpoint_[i+j].x << ", " << global_setpoint_[i+j].y << ", " << global_setpoint_[i+j].z << std::endl;
                opt_points[j].transforms.push_back(tf_[j]); //push back the transform
                opt_points[j].velocities.push_back(vel_); //push back the velocity
                opt_points[j].accelerations.push_back(acc_); //push back the acceleration
                std::cout << "Local setpoint " << j << ": " << std::endl;
                std::cout << opt_points[j].transforms[0].translation.x << ", " << opt_points[j].transforms[0].translation.y << ", " << opt_points[j].transforms[0].translation.z << std::endl;
                //std::cout << "Hello World 1" << std::endl;
                if(tf_[j].translation.x == 16 && tf_[j].translation.y == 0) {
                    tf_[j].rotation.x = 0;
                    tf_[j].rotation.y = 0;
                    tf_[j].rotation.z = 0.7071068;
                    tf_[j].rotation.w = 0.7071068;
                    opt_points[j].transforms.push_back(tf_[j]); //push back the transform
                }
            }
        }
        if(54 < i && i < 64) {
            for(int j = 0; j < 10; j++) {
                if(i+j <= 63) {
                tf_[j].translation.x = global_setpoint_[i+j].x;
                tf_[j].translation.y = global_setpoint_[i+j].y;
                tf_[j].translation.z = global_setpoint_[i+j].z;
                std::cout << "Global setpoint " << i+j << ": " << std::endl;
                std::cout << global_setpoint_[i+j].x << ", " << global_setpoint_[i+j].y << ", " << global_setpoint_[i+j].z << std::endl;
                //std::cout << "Hello World 2" << std::endl;
                }
                if(i+j > 63) {
                tf_[j].translation.x = global_setpoint_[0].x;
                tf_[j].translation.y = global_setpoint_[0].y;
                tf_[j].translation.z = global_setpoint_[0].z;
                }
                opt_points[j].transforms.push_back(tf_[j]); //push back the transform
                opt_points[j].velocities.push_back(vel_); //push back the velocity
                opt_points[j].accelerations.push_back(acc_); //push back the acceleration
                if(tf_[j].translation.x == 16 && tf_[j].translation.y == 0) {
                    tf_[j].rotation.x = 0;
                    tf_[j].rotation.y = 0;
                    tf_[j].rotation.z = 0.7071068;
                    tf_[j].rotation.w = 0.7071068;
                    opt_points[j].transforms.push_back(tf_[j]); //push back the transform
                }
            }
        }
        opt_points[0].transforms[0].translation = tf_[0].translation; //opt_points[0].transforms.assign = 
        opt_points[0].transforms[0].rotation = tf_[0].rotation;
        std::cout << "-->Local transform " << 0 << ": " << std::endl;
        std::cout << tf_[0].translation.x << ", " << tf_[0].translation.y << ", " << tf_[0].translation.z << std::endl;
        //control_point_ = opt_points[0];
        //setpoint_pub_.publish(control_point);
        for(int j = 0; j < 10; j++) {
            traj_msg_.points.push_back(opt_points[j]);
        }
        traj_msg_.header.stamp = ros::Time::now();
        //ROS_INFO_STREAM_ONCE(traj_msg_);
        //ROS_INFO_STREAM(pos_target_);
        //mavros_pub_.publish(pos_target_);
        traj_msg_.header.stamp = ros::Time::now();
        //traj_pub_.publish(traj_msg_);
        //opt_point_pub_.publish(opt_points[0]);
        std::cout << "The control point is: ";
        std::cout << opt_points[0].transforms[0].translation.x << opt_points[0].transforms[0].translation.y << opt_points[0].transforms[0].translation.z << std::endl;
        target_enu_pose_.header.stamp = ros::Time::now();
                //traj_pub_.publish(traj_msg_);
        // pos_target_.header.stamp = ros::Time::now();
        // pos_target_.yaw = tf::getYaw(opt_points[0].transforms[0].rotation);

        target_enu_pose_.pose.position.x = opt_points[0].transforms[0].translation.x;
        target_enu_pose_.pose.position.y = opt_points[0].transforms[0].translation.y;
        target_enu_pose_.pose.position.z = opt_points[0].transforms[0].translation.z;


        //mavros_pub_.publish(pos_target_);

        // ROS_INFO_STREAM(target_enu_pose_);
        // ROS_INFO_STREAM(current_odom_);
        //std::cout<<"Hello World";
        setpoint_p_pub_.publish(target_enu_pose_);
        //target_reached = checkPositionError(geo_error_, targetTransfer(global_setpoint_[i].x, global_setpoint_[i].y, global_setpoint_[i].z)); //check if drone reaches target or not
        //std::cout<< global_setpoint_[i].x << global_setpoint_[i].y << global_setpoint_[i].z;
        target_reached = checkPositionError(geo_error_, targetTransfer(opt_points[0].transforms[0].translation.x, opt_points[0].transforms[0].translation.y, opt_points[0].transforms[0].translation.z)); 
        if(target_reached) {
            std::cout << " i = " << i << std::endl;
            i++; //increase i after the drone reaches the target
            std::cout << "i++ = " << i << std::endl;
        }
        if(i==64) {
            final_target_reached = true; //stop the loop when the final target is reached
            landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
        }
        //ROS_INFO_STREAM_ONCE("ROS is spinning.");
        ros::spinOnce();
        rate.sleep();
    }
}

bool MultiDOFControl::checkPositionError(double geo_error, geometry_msgs::PoseStamped target_position) {
    Eigen::Vector3d error;
    error << target_position.pose.position.x - current_odom_.pose.pose.position.x, target_position.pose.position.y - current_odom_.pose.pose.position.y, target_position.pose.position.z - current_odom_.pose.pose.position.z;
    return (error.norm() <= geo_error) ? true : false;
}

void MultiDOFControl::distanceBetween(geometry_msgs::PoseStamped current_position, geometry_msgs::PoseStamped target_position) {
    Eigen::Vector3d distance;
    distance << current_position.pose.position.x - target_position.pose.position.x, current_position.pose.position.y - target_position.pose.position.y,current_position.pose.position.z - target_position.pose.position.z;
    std::cout << "Distance: " << distance.norm() << "\n";
}

geometry_msgs::PoseStamped MultiDOFControl::targetTransfer(double x, double y, double z) {
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}

/* calculate components of velocity about x, y, z axis
   input: desired velocity, current and target poses (ENU) */
geometry_msgs::Vector3 MultiDOFControl::velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target) {
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double zc = current.pose.position.z;

    double xt = target.pose.position.x;
    double yt = target.pose.position.y;
    double zt = target.pose.position.z;

    double dx = xt - xc;
    double dy = yt - yc;
    double dz = zt - zc;

    double d = sqrt(sqr(dx) + sqr(dy) + sqr(dz));

    geometry_msgs::Vector3 vel;

    vel.x = ((dx / d) * v_desired);
    vel.y = ((dy / d) * v_desired);
    vel.z = ((dz / d) * v_desired);

    return vel;
}



/* perform takeoff task
   input: setpoint to takeoff and hover time */
void MultiDOFControl::takeOff(geometry_msgs::PoseStamped setpoint, double hover_time) {
    ros::Rate rate(10.0);
    std::printf("\n[ INFO] Takeoff to [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
    bool takeoff_reached = false;
    while (ros::ok() && !takeoff_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_p_pub_.publish(target_enu_pose_);

        takeoff_reached = checkPositionError(geo_error_, setpoint);
        if (takeoff_reached) {
            hovering(setpoint, hover_time);
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* perform hover task
   input: setpoint to hover and hover time */
void MultiDOFControl::hovering(geometry_msgs::PoseStamped setpoint, double hover_time) {
    ros::Rate rate(10.0);
    ros::Time t_check;

    std::printf("\n[ INFO] Hovering at [%.1f, %.1f, %.1f] in %.1f (s)\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z, hover_time);
    t_check = ros::Time::now();
    while ((ros::Time::now() - t_check) < ros::Duration(hover_time)) {
        setpoint_p_pub_.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }
}


/* perform land task
   input: set point to land (e.g., [x, y, 0.0]) */
void MultiDOFControl::landing(geometry_msgs::PoseStamped setpoint) {
     //std::cout << "In landing()" << std::endl;
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Landing\n");
    while (ros::ok() && !land_reached) {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_p_pub_.publish(target_enu_pose_);

        land_reached = checkPositionError(land_error_, setpoint);

        if (current_state_.system_status == 3) {
            std::printf("\n[ INFO] Land detected\n");
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                break;
            }
        }
        else if (land_reached) {
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent) {
                std::printf("\n[ INFO] LANDED\n");
            }
        }
        else {
            ros::spinOnce();
            rate.sleep();
        }
    }

    operation_time_2_ = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2_ - operation_time_1_).toSec());
    ros::shutdown();
}
