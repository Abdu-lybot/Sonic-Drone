//
// Created by lybot on 13/4/22.
//

//#include "sonic_v2.h"
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
#include "listener/pointService.h"
//#include <mavros/mavros_plugin.h>
#define PI 3.14
#define GOAL_POSITION_MARGIN 0.20
#define GOAL_YAW_MARGIN 10

using namespace std;
bool moving_status = true;

mavros_msgs::State current_state;
static float gps_x, gps_y, gps_z, gps_ori_x, gps_ori_y, gps_ori_z, gps_ori_w;
static float vslam_x, vslam_y, vslam_z, vslam_ori_x, vslam_ori_y, vslam_ori_z, vslam_ori_w;
static double key_throttle, key_pitch, key_gripper, roll, pitch, yaw, yaw_degrees;
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
    yaw_degrees = yaw * 180.0 / PI;
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
    float brake_left = 1730;
    float brake_neutral = 1180;
    float brake_right = 1750;
    int pitch = 1500; //1800
    //int throttle = 0;//1100; //1200
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
    ros::ServiceClient client = nh.serviceClient<listener::pointService>("plannerService");
    listener::pointService srv;


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

    string cw_or_ccw(double yaw_diff){
        if (yaw_diff > 180) yaw_diff -= 360;
        if (yaw_diff <= -180) yaw_diff += 360;
        if (yaw_diff > 0){
            ROS_INFO_NAMED("Turn"," CCW is closer to goal");
            return "ccw"; //positive
        }else{
            ROS_INFO_NAMED("Turn"," CW is closer to goal");
            return "cw"; //negative
        }
    }
    bool in_margin_angle(double yaw_diff){
            if(abs(yaw_diff) > GOAL_YAW_MARGIN){
                return true;
                }
            else{
                ROS_ERROR("SONIC OUT OF MARGIN");
                return false;
            }
    };
    float get_distance(geometry_msgs::Point goal){
        float d = sqrt(pow(goal.x - vslam_x,2) + pow(goal.y - vslam_y,2));
        return d;
    }
    bool in_margin_dist(geometry_msgs::Point goal){

        if (get_distance(goal) <  GOAL_POSITION_MARGIN) {
            ROS_ERROR("Goal Reached");
            return true;
        }
        else{
            ROS_ERROR("Rolling Forward, distance left: %f", get_distance(goal));
            return false;
        }
    }

    void forward(geometry_msgs::Point goal){
        ROS_INFO("Sonic is moving forward. Goal: (x:%lf, y:%lf). Current: (x:%lf, y:%lf).", goal.x, goal.y, vslam_x, vslam_y);
        manual_move(0, pitch, key_throttle, 000, 0, brake_neutral, brake_neutral, steering_forward,0);
    }

    void turnccw(double angle){
        ROS_INFO("Sonic is moving left. Goal: %lf. Current: %lf.", angle, yaw_degrees);
        manual_move(0, pitch, key_throttle, 000, 0, brake_neutral, brake_left, steering_left,0);
    }

    void turncw(double angle){
        ROS_INFO("Sonic is moving right. Goal: %lf. Current: %lf.", angle, yaw_degrees);
        manual_move(0, pitch, key_throttle, 000, 0, brake_right, brake_neutral, steering_right,0);
    }

    void roll_turn(float angle, ros::Rate rate){
        double angle_360 = angle < 0 ? angle + 360 : angle;
        double yaw_degree_360 = yaw_degrees < 0 ? yaw_degrees + 360 : yaw_degrees;
        double yaw_diff = angle_360 - yaw_degree_360;
        while (in_margin_angle(yaw_diff)){
            if (cw_or_ccw(yaw_diff) == "ccw"){
                turnccw(angle);
                rate.sleep();
                ros::spinOnce();
            }
            else {
                turncw(angle);
                rate.sleep();
                ros::spinOnce();
            }
            yaw_degree_360 = yaw_degrees < 0 ? yaw_degrees + 360 : yaw_degrees;
            yaw_diff = angle_360 - yaw_degree_360;
        }
    }

    bool roll_forw(geometry_msgs::Point goal, ros::Rate rate, double starting_angle){
//        double angle_360 = starting_angle < 0 ? starting_angle + 360 : starting_angle;
//        double yaw_degree_360 = yaw_degrees < 0 ? yaw_degrees + 360 : yaw_degrees;
//        double yaw_diff = angle_360 - yaw_degree_360;
        while(!in_margin_dist(goal)){
//            yaw_degree_360 = yaw_degrees < 0 ? yaw_degrees + 360 : yaw_degrees;
//            yaw_diff = angle_360 - yaw_degree_360;
//            if (!in_margin_angle(yaw_diff)) {
//                cout << "NOT CORRECT" << endl;
//                return false;
//            }
//            if (yaw_degrees)
            forward(goal);
            rate.sleep();
            ros::spinOnce();
        }
        return true;
    }

    void warming_engines(ros::Rate rate){
            ros::Time last_request = ros::Time::now();
            while((ros::Time::now() - last_request < ros::Duration(4.0)) && moving_status) {
                ROS_INFO("Warming Engines");
                manual_move(0, 0, 0, 000, 0, brake_neutral, brake_neutral, steering_forward,0);
                usleep(400000);
                manual_move(0, 0, 0, 000, 0, brake_right, brake_neutral, steering_left,0);
                usleep(400000);
                manual_move(0, 0, 0, 000, 0, brake_neutral, brake_left, steering_right,0);
                usleep(400000);
                manual_move(0, 0, 0, 000, 0, brake_neutral, brake_neutral, steering_forward,0);
                rate.sleep();
            }
    };
    void stop_roll(ros::Rate rate){
        ros::Time last_request = ros::Time::now();
            while((ros::Time::now() - last_request < ros::Duration(3.0)) && moving_status) {
                ROS_INFO("Stopping");
                manual_move(0, 0, 300, 000, 0, brake_right, brake_left, steering_forward,0);
                rate.sleep();
            }
    };
    void rest(){
        ROS_WARN("Resting motors to avoid burning");
        manual_move(0, 0, 0, 000, 0, brake_neutral, brake_neutral, steering_forward,0);
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
        ros::Time last_request = ros::Time::now();
        offb_set_mode.request.custom_mode = flightmode;
        mavros_msgs::CommandBool arm_cmd;
        while((ros::Time::now() - last_request < ros::Duration(100.0)) && moving_status){
            set_mode_client.call(offb_set_mode);
            arming_client.call(set_armed(&arm_cmd, true));
            usleep(100000);
        }
    }
};


int main(int argc, char **argv)
{
    cout << "Sonic V2 Library "<<endl;

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
            sonic.warming_engines(rate);
//            geometry_msgs::Point p;
//            while (ros::ok()){
//                sonic.forward(p);
//                rate.sleep();
//                ros::spinOnce();
//            }
            sonic.srv.request.ready = true;
            ros::service::waitForService("plannerService", ros::Duration(5));

            geometry_msgs::Point goal;
            double dif_x, dif_y, angle;
            bool correct = false;
            while (sonic.client.call(sonic.srv)) {
//                if(sonic.srv.response.again) continue;

//                do {
                    goal = sonic.srv.response.point;
                    dif_x = goal.x - vslam_x;
                    dif_y = goal.y - vslam_y;
                    angle = atan2(dif_y, dif_x) * 180.0 / PI;

                    sonic.roll_turn(angle, rate);
                    sonic.stop_roll(rate);
                    correct = sonic.roll_forw(goal, rate, angle);
                    sonic.stop_roll(rate);
//                } while (!correct);

            }
//                double angle = 45;
            //sonic.roll_turn(angle, rate);
            sonic.stop_roll(rate);
//            geometry_msgs::Point goal; goal.x = 0.2; goal.y = 0;
//            sonic.roll_forw(goal, rate);
//            sonic.stop_roll(rate);
//            ROS_WARN("GOAL 2");
//            goal.x = 0.3; goal.y = 0;
//            sonic.roll_forw(goal, rate);
//            sonic.stop_roll(rate);
            sonic.rest();

//                              for keyboard
//                last_request = ros::Time::now();
//                while((ros::Time::now() - last_request < ros::Duration(6.0)) && moving_status) {
//                    //cout << "Throttle : "<<key_throttle<<", Pitch : "<<key_pitch<<", Gripper : "<<key_gripper<<endl;
//                    //sonic.manual_move(0, key_pitch, key_throttle, 000, 0, 0, 0, key_gripper, 0);
//                    sonic.turnccw(50);
//                    rate.sleep();
//                    ros::spinOnce();
//                }
            ROS_WARN("Function Ended");
            sonic.set_moving_status(false);
//              sonic.set_armed(&arm_cmd, false);
            ros::spin();
        }
    }
    return 0;
}