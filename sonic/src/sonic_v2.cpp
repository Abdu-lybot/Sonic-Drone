//
// Created by lybot on 13/4/22.
//

#include "sonic_v2.h"
//
// Created by lybot on 11/4/22.
//

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <nav_msgs/Odometry.h>
//#include <mavros/mavros_plugin.h>
#define PI 3.14

using namespace std;
bool moving_status = true;

mavros_msgs::State current_state;
static float gps_x, gps_y, gps_z, gps_ori_x, gps_ori_y, gps_ori_z, gps_ori_w;
static float vslam_x, vslam_y, vslam_z, vslam_ori_x, vslam_ori_y, vslam_ori_z, vslam_ori_w;
static double key_throttle, key_pitch, key_gripper, roll, pitch, yaw;
static string flightmode;

void cb_keyboard(geometry_msgs::Point msg){
    key_throttle = msg.x;
    key_pitch = msg.y;
    key_gripper = msg.z;

}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    //cout << current_state.armed <<endl;
}
void gps_cb(nav_msgs::Odometry data){
    gps_x = data.pose.pose.position.x;
    gps_y = data.pose.pose.position.y;
    gps_z = data.pose.pose.position.z;
    gps_ori_x = data.pose.pose.orientation.x;
    gps_ori_y = data.pose.pose.orientation.y;
    gps_ori_z = data.pose.pose.orientation.z;
    gps_ori_w = data.pose.pose.orientation.w;
    cout << "GPS info" <<endl;
    cout << " x : " << gps_x << " y : " << gps_y << " z : "<< gps_z<< endl;
    cout << " ori_x : " << gps_ori_x << " ori_y : "<< gps_ori_y << " ori_z : " << gps_ori_z << " ori_w" << gps_ori_w <<endl;

}
void vslam_cb(nav_msgs::Odometry data){
    vslam_x = data.pose.pose.position.x;
    vslam_y = data.pose.pose.position.y;
    vslam_z = data.pose.pose.position.z;
    vslam_ori_x = data.pose.pose.orientation.x;
    vslam_ori_y = data.pose.pose.orientation.y;
    vslam_ori_z = data.pose.pose.orientation.z;
    vslam_ori_w = data.pose.pose.orientation.w;
    tf::Quaternion q(vslam_ori_x, vslam_ori_y, vslam_ori_z, vslam_ori_w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
//    cout << "VSLAM info" <<endl;
//    cout << " x : " << vslam_x << " y : " << vslam_y << " z : "<< vslam_z<< endl;
//    cout << " roll : " << roll << " pitch : "<< pitch << " yaw : " << yaw <<endl;
}

class SonicFirmware{
public:
    bool disarm {false};
    float steering_left = 1180;
    float steering_forward = 1500;
    float steering_right = 1840;
    float brake_left = 1650;
    float brake_neutral = 1180;
    float brake_right = 1650;
    ros::NodeHandle nh;
    ros::Subscriber keyboard_inp = nh.subscribe<geometry_msgs::Point>("keyboard_sonic",1, cb_keyboard);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 20, state_cb);
    ros::Subscriber gps_pose = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local",10 ,gps_cb);
    ros::Subscriber vslam_pose = nh.subscribe<nav_msgs::Odometry>
            ("camera/odom/sample",10 ,vslam_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher manual_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 1000);


    mavros_msgs::AttitudeTarget construct_message(float thr, float pitch, float yaw){
        tf::Quaternion quat;
        quat.setRPY(0,pitch,yaw);
        mavros_msgs::AttitudeTarget _move = mavros_msgs::AttitudeTarget();
        _move.header.stamp = ros::Time::now();
//        _move.type_mask = 7;
        _move.body_rate.x = 1500;
        _move.body_rate.y = 0;
        _move.body_rate.z = 0;
        _move.orientation.x = quat.x();
        _move.orientation.y = quat.y();
        _move.orientation.z = quat.z();
        _move.orientation.w = quat.w();
        cout << "Ori. in Quat = x,y,z,w " << _move.orientation.x<<","<<_move.orientation.y<<","<<_move.orientation.z <<","<<_move.orientation.w <<endl;
        _move.thrust = thr;
        return _move;
    }

    void manual_move(int roll, int pitch, int throttle, int yaw, int mode, int right_brake, int left_brake, int steering, int servo4){
        mavros_msgs::OverrideRCIn cmd;
        std::vector<uint16_t> raw_rc_in;
        cmd.channels[0] = roll;
        cmd.channels[1] = pitch;
        cmd.channels[2] = throttle;
        cmd.channels[3] = yaw;
        cmd.channels[4] = 0;
        cmd.channels[5] = right_brake;
        cmd.channels[6] = left_brake;
        cmd.channels[7] = steering;
        cmd.channels[8] = 0;
        cmd.channels[9] = 0;
        cmd.channels[10] = 0;
        cmd.channels[11] = 0;
        cmd.channels[12] = 0;
        cmd.channels[13] = 0;
        cmd.channels[14] = 0;
        cmd.channels[15] = 0;
        cmd.channels[16] = 0;
        auto rcin_msg = boost::make_shared<mavros_msgs::RCIn>();
        manual_pub.publish(cmd);
    }

    void forward(float distance){
        ROS_INFO("Sonic will move forward with %f from %lf", distance, vslam_x);
        manual_move(0, 0, 0, 000, 0, brake_neutral, brake_neutral, steering_forward,0);

    }

    void turnccw(float angle){
        ROS_INFO("Sonic will move left with %f from %lf ", angle, yaw);
        manual_move(0, 0, 0, 000, 0, brake_neutral, brake_left, steering_left,0);

    }

    void turncw(float angle){
        ROS_INFO("Sonic will move right with %f from %lf ", angle, yaw);
        manual_move(0, 0, 0, 000, 0, brake_right, brake_neutral, steering_right,0);

    }

    void set_moving_status(bool stopped = false){
        moving_status = stopped;
        //cout << "The drone should stop!" << endl;
    }
    mavros_msgs::CommandBool &set_armed(mavros_msgs::CommandBool *arm_cmd, bool arm){
        //cout << "ARMING ";
        //cout << boolalpha;
        //cout <<arm<<endl;
        arm_cmd->request.value = arm;
        if(!arm){
            disarm = true;
        }else{
            disarm = false;
        }
        return *arm_cmd;
    }
    unsigned char get_armed_status(mavros_msgs::CommandBool arm_cmd){
        cout << "status "<< arm_cmd.response.success <<endl;
        return arm_cmd.response.success;
    }
    void keep_armed(){
        mavros_msgs::SetMode offb_set_mode;
        //offb_set_mode.request.custom_mode = "GUIDED_NOGPS";
        ros::Time last_request = ros::Time::now();
        offb_set_mode.request.custom_mode = flightmode;
        mavros_msgs::CommandBool arm_cmd;
        while((ros::Time::now() - last_request < ros::Duration(100.0)) && moving_status){
            set_mode_client.call(offb_set_mode);
            //ROS_INFO("GUIDED_NOGPS enabled");
            arming_client.call(set_armed(&arm_cmd, true));
            //ROS_INFO("Vehicle armed");
            usleep(100000);
        }
    }
};


int main(int argc, char **argv)
{
    cout << "SOnic V2 Library "<<endl;

    ros::init(argc, argv, "Sonic_Drone");
    flightmode = "ACRO";  //STABILIZE
    SonicFirmware sonic;
    ros::Rate rate(20.0);

    //the setpoint publishing rate MUST be faster than 2Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok()){

        std::thread th(&SonicFirmware::keep_armed,sonic);
        if(moving_status){
                sleep(1);
                ros::Time last_request = ros::Time::now();

                while((ros::Time::now() - last_request < ros::Duration(3.0)) && moving_status) {
                    ROS_INFO("Warming Engines");
                    sonic.manual_move(0, 000, 000, 000, 0, 000, 000, 000, 000);
                    rate.sleep();
                }
                last_request = ros::Time::now();

                while((ros::Time::now() - last_request < ros::Duration(6.0)) && moving_status) {
                    //cout << "Throttle : "<<key_throttle<<", Pitch : "<<key_pitch<<", Gripper : "<<key_gripper<<endl;  //for keyboard
                    //sonic.manual_move(0, key_pitch, key_throttle, 000, 0, 0, 0, key_gripper, 0);
                    sonic.turnccw(50);
                    rate.sleep();
                    ros::spinOnce();
                }
                last_request = ros::Time::now();

                while((ros::Time::now() - last_request < ros::Duration(6.0)) && moving_status) {
                    sonic.turncw(-50);
                    rate.sleep();
                    ros::spinOnce();
                }
                last_request = ros::Time::now();

                while((ros::Time::now() - last_request < ros::Duration(6.0)) && moving_status) {
                    sonic.forward(0.5);
                    rate.sleep();
                    ros::spinOnce();
                }
                last_request = ros::Time::now();

                 while((ros::Time::now() - last_request < ros::Duration(3.0)) && moving_status) {
                    ROS_INFO("stopping");
                    sonic.manual_move(0, 0, 0000, 0, 0, 0, 0000, 000, 000);
                    rate.sleep();
                }
                ROS_WARN("Function Ended");
                sonic.set_moving_status(false);
//              sonic.set_armed(&arm_cmd, false);
                ros::spin();
            }
    }
    return 0;
}