#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <wplive/pid.hpp>

double dt = 1/32.0;

enum journey {takeoff, firstWP, secondWP, thirdWP, fourthWP, landing};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

double measure_dist(double form_x, double form_y, double form_z, double dest_x, double dest_y, double dest_z){
    double distance_x = dest_x - form_x;
    double distance_y = dest_y - form_y;
    double distance_z = dest_z - form_z;
    double jarak_tridi = sqrt(pow(distance_x,2) + pow(distance_y,2) + pow(distance_z,2));
    return jarak_tridi;
}

// geometry_msgs::Twist assignPID(double wp_x, double wp_y, double wp_z, PID mypid_x, PID mypid_y, PID mypid_z){
//     geometry_msgs::Twist vel;
//     vel.linear.x = mypid_x.calculate(wp_x, current_position.pose.position.x);
//     vel.linear.y = mypid_y.calculate(wp_y, current_position.pose.position.y);
//     vel.linear.z = mypid_z.calculate(wp_z, current_position.pose.position.z);
//     vel.angular.x = 0;
//     vel.angular.y = 0;
//     vel.angular.z = 0;
//     return vel;
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "pid_node_new");
    ros::NodeHandle nh;

    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>  // publish setpoint_velocity
        ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>  // publish setpoint_position
        ("mavros/setpoint_position/local", 10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>  // subscribe position
        ("mavros/local_position/pose", 10, local_pos_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    double max_speed = 0.5;
    std::cout << "input maximum speed (0 < v < 1), 0.5 recommended : ";
    std::cin >> max_speed;
    // PID mypid_z_takeoff(dt, max_speed, -max_speed, 1.000, 0.355, 0.015);
    PID mypid_x(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_y(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_z(dt, max_speed, -max_speed, 1.0, 0.6, 0);  // aman
    // p tambah cepet
    // d overshoot


    std::cout << "START MISSION\n";
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1/dt);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && (current_state.mode != "OFFBOARD" || !current_state.armed)){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);  // ini keknya hanya pancingan, waypoint buat takeoff harus buat lagi
        ros::spinOnce();
        rate.sleep();
    }

    // velocity for takeoff
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    double wp[6][3] = {
        0, 0, 0,
        0, 0, 1.5,
        4, 0, 1.5,
        4, 4, 2,
        2, 2, 2,
        0, 0, 2,
    };

    ros::Time t0 = ros::Time::now();
    ros::Time forDelay = ros::Time::now();
    journey drone = takeoff;
    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        // takeoff
        if((drone==takeoff) && (ros::Time::now()-forDelay > ros::Duration(4))){
            ROS_INFO("takeoff");
            vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);
            if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[1][0], wp[1][1], wp[1][2]) < 0.2){
                drone = firstWP;
                forDelay = ros::Time::now();
            }
        // firstWP
        } else if(drone==firstWP){
            if(ros::Time::now()-forDelay > ros::Duration(4)){
                ROS_INFO("firstWP");
                vel.linear.x = mypid_x.calculate(wp[2][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[2][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);
                if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[2][0], wp[2][1], wp[2][2]) < 0.2){
                    drone = secondWP;
                    forDelay = ros::Time::now();
                }
            } else{
                vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);
            }
        // secondWP
        } else if(drone==secondWP){
            if(ros::Time::now()-forDelay > ros::Duration(4)){
                ROS_INFO("secondWP");
                vel.linear.x = mypid_x.calculate(wp[3][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[3][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[3][2], current_position.pose.position.z);
                if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[3][0], wp[3][1], wp[3][2]) < 0.2){
                    drone = thirdWP;
                    forDelay = ros::Time::now();
                }
            } else{
                vel.linear.x = mypid_x.calculate(wp[2][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[2][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[2][2], current_position.pose.position.z);
            }
        // thirdWP
        } else if(drone==thirdWP){
            if(ros::Time::now()-forDelay > ros::Duration(4)){
                ROS_INFO("thirdWP");
                vel.linear.x = mypid_x.calculate(wp[4][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[4][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[4][2], current_position.pose.position.z);
                if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[4][0], wp[4][1], wp[4][2]) < 0.2){
                    drone = fourthWP;
                    forDelay = ros::Time::now();
                }
            } else{
                vel.linear.x = mypid_x.calculate(wp[3][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[3][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[3][2], current_position.pose.position.z);
            }
        // fourthWP
        } else if(drone==fourthWP){
            if(ros::Time::now()-forDelay > ros::Duration(4)){
                ROS_INFO("fourthWP");
                vel.linear.x = mypid_x.calculate(wp[5][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[5][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[5][2], current_position.pose.position.z);
                if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[1][0], wp[1][1], wp[1][2]) < 0.2){
                    drone = FifthWP;
                    forDelay = ros::Time::now();
                }
            } else{
                vel.linear.x = mypid_x.calculate(wp[4][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[4][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[4][2], current_position.pose.position.z);
            }
        //FifthWP
            } else if(drone==FifthWP){
            if(ros::Time::now()-forDelay > ros::Duration(4)){
                ROS_INFO("fifthWP");
                vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);
                if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[5][0], wp[5][1], wp[5][2]) < 0.2){
                    drone = landing;
                    forDelay = ros::Time::now();
                }
            } else{
                vel.linear.x = mypid_x.calculate(wp[5][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[5][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[5][2], current_position.pose.position.z);
            }
        // landing
        } else if(drone==landing){
            if(ros::Time::now()-forDelay > ros::Duration(4)){
                ROS_INFO("landing");
                vel.linear.x = mypid_x.calculate(wp[0][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[0][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[0][2], current_position.pose.position.z);
                if(measure_dist(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z, wp[0][0], wp[0][1], wp[0][2]) < 0.2){
                    break;
                }
            } else{
                vel.linear.x = mypid_x.calculate(wp[1][0], current_position.pose.position.x);
                vel.linear.y = mypid_y.calculate(wp[1][1], current_position.pose.position.y);
                vel.linear.z = mypid_z.calculate(wp[1][2], current_position.pose.position.z);
            }
        }

        ROS_INFO("x = %f, y = %f, z = %f", current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z);
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time t1 = ros::Time::now();
    ROS_INFO("time measured : %lf", (t1-t0).toSec());

    return 0;
}
