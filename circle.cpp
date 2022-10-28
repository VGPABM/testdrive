#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <trialwp/pid.hpp>

#define PI 3.14159265

double dt = 1/32.0;

struct mission{
    double x;
    double y;
    double z;
    bool done;
};

struct mission circle[4]{
    {2.5 , 2.5, 2, false},
    {5, 0, 2, false},
    {2.5, -2.5, 2, false}
};

 // velocity for takeoff
geometry_msgs::Twist vel;

ros::Time t0 = ros::Time::now();
ros::Time forDelay = ros::Time::now();

bool near_equal(double a, double b, double precision)
{
    return (std::max(fabs(a), fabs(b)) - std::min(fabs(a), fabs(b))) < precision;
}

void goTakeOff(ros::Publisher cmd_pub, ros::Rate rate,double max_speed)
{
    geometry_msgs::Twist vel;
    bool tookOff = false;
    ros::Time takingoff;
        PID mypid_x(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_y(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_z(dt, max_speed, -max_speed, 1.0, 0.6, 0);  // aman
    // Looping hingga took off
    while(ros::ok() && !tookOff)
    {
        ros::spinOnce();
        rate.sleep();

        // Jika belum mendekati z => 6 maka gerak ke z => 6
        if(!near_equal(current_position.pose.position.z, 6, 0.3))
        {
            vel.linear.x = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(6, current_position.pose.position.z);

            cmd_pub.publish(vel);
            continue;
        }

        // Mulai perhitungan waktu
        if(takingoff.is_zero())
        {
            takingoff = ros::Time::now();
            continue;
        }

        cmd_pub.publish(vel);

        if(ros::Time::now() - takingoff < ros::Duration(5.0))
        {
            continue;
        }

        // Jika sudah melewati 5 detik maka sudah bisa dianggap stabil dan sudah take off
        tookOff = true;
    }
}

void goLand(ros::Publisher cmd_pub, ros::Rate rate,double max_speed)
{
    PID mypid_x(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_y(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_z(dt, max_speed, -max_speed, 1.0, 0.6, 0);  // aman
    ros::Time landing;
    geometry_msgs::Twist vel;
    bool landed = false;

    // Looping hingga landed
    while(ros::ok() && !landed)
    {
        ros::spinOnce();
        rate.sleep();

        // Jika belum mendekati posisi x,y => 0
        if(!near_equal(current_position.pose.position.x, 0, 0.3) || !near_equal(current_position.pose.position.y, 0, 0.3))
        {
            vel.linear.x = mypid_x.calculate(0, current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(0, current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(current_position.pose.position.z, current_position.pose.position.z);

            // Mulai pindahin mendekati posisi x,y => 0
            cmd_pub.publish(vel);
            continue;
        }

        // Mengecek apakah sudah mulai hitung waktu
        if(landing.is_zero())
        {
            landing = ros::Time::now();
            continue;
        }
            
        cmd_pub.publish(vel);

        // Jika sudah melewati 5 detik maka mulai untuk landing ke z => 0
        if(ros::Time::now() - landing < ros::Duration(5.0))
        {
            continue;
        }

        // Bergerak ke z => 0 jika belum mendekati
        if(!near_equal(current_position.pose.position.z, 0, 0.3))
        {
            vel.linear.x = mypid_x.calculate(current_position.pose.position.x, current_position.pose.position.x);
            vel.linear.y = mypid_y.calculate(current_position.pose.position.y, current_position.pose.position.y);
            vel.linear.z = mypid_z.calculate(0, current_position.pose.position.z);

            cmd_pub.publish(vel);
            // ROS_INFO("Going Home! | Going to z:%f", pose.pose.position.z);
            continue;
        } else {
            // Jika sudah mendekati titik z =>0 maka sudah bisa dianggap landed
            landed = true;
        }
    }
}


void goMission(ros::Publisher cmd_pub, ros::Rate rate, double max_speed, double diameter){
    ros::Time empty;
    ros::Time misi;
    geometry_msgs::Twist vel;
    PID mypid_x(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_y(dt, max_speed, -max_speed, 1.5, 0.15, 0);
    PID mypid_z(dt, max_speed, -max_speed, 1.0, 0.6, 0);  // aman
    double start_positionx = current_position.pose.position.x;
    double start_positiony = current_position.pose.position.y;
    double radius = diameter / 2;
    double cx = 0;
    double cy = radius;
    //int missionDone = 0;
    //int totalMission = (sizeof(circle) / sizeof(mission[0]));
    
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        double degree, tujuanx,tujuany,vx,vy;
        double Tsatu = atan2(current_position.pose.position.x,current_position.pose.position.y);
        double Tdua = 2*PI - Tsatu;
        vx = max_speed *sin(Tdua*180/PI);
        vy = max_speed *cos(Tdua*180/PI);
        tujuanx = current_position.pose.position.x + vx;
        tujuany = current_position.pose.position.y + vy;
        vel.linear.x = mypid_x.calculate(tujuanx,current_position.pose.position.x);
        vel.linear.y = mypid_y.calculate(tujuany,current_position.pose.position.y);
        cmd_pub.publish(vel);
    } 
    
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
current_position = *msg;
}



int main(int argc, char**argcv){
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
    double diameter = 5;
    std::cout << "input maximum speed (0 < v < 1), 0.5 recommended : ";
    std::cin >> max_speed;
    std::cout << "input diameter to make, 5 recommended :";
    std::cin >> diameter;
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
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(4))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(4))){
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


      // Mulai take off
    ROS_INFO("Taking Off! | Break a leg!");
    goTakeOff(cmd_pub, rate,max_speed);

     // Mulai jalankan misi
    ROS_INFO("Starting Mission! | Finger crossed!");
    goMission(cmd_pub, rate, max_speed,diameter);

    // Mulai kembali ke titik awal
    ROS_INFO("Going Home! | Yay");
    goLand(cmd_pub, rate,max_speed);

    ROS_INFO("Mission Done! | Woohoo!");
    return 0;


}
