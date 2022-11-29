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

#define PI 3.14

using namespace std;
bool moving_status = true;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    //cout << current_state.armed <<endl;
}

class SonicFirmware{
public:
    bool disarm {false};
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    mavros_msgs::AttitudeTarget construct_message(float thr, float pitch, float yaw){
        tf::Quaternion quat;
        quat.setRPY(0,pitch,yaw);
        mavros_msgs::AttitudeTarget _move = mavros_msgs::AttitudeTarget();
        _move.header.stamp = ros::Time::now();
        _move.type_mask = 7;
//        _move.body_rate.x = 0;
//        _move.body_rate.y = 0;
//        _move.body_rate.z = 0;
        _move.orientation.x = quat.x();
        _move.orientation.y = quat.y();
        _move.orientation.z = quat.z();
        _move.orientation.w = quat.w();
        _move.thrust = thr;
        return _move;

    }

    void forward(float thrust, float pitch, int time, ros::Rate rate){
        ros::Time last_requested = ros::Time::now();
        while(ros::Time::now() - last_requested < ros::Duration(time) && ros::ok()){
            cout << "Moving with Thrust "<< thrust << " with time = "<<time<<endl;
            local_pos_pub.publish(construct_message(thrust, pitch, 0));
            ros::spinOnce();
            rate.sleep();
        }
    }
    void turn(float thrust, float yaw, int time, ros::Rate rate){
        ros::Time last_requested = ros::Time::now();
        while(ros::Time::now() - last_requested < ros::Duration(time) && ros::ok()){
            cout << "Moving with Thrust "<< thrust << " with time = "<<time<<endl;
            local_pos_pub.publish(construct_message(thrust, 0, yaw));
            ros::spinOnce();
            rate.sleep();
        }
    }
    double input;
    double acum {0.5};

    double thrust_keyboard(){
        cout <<"input keayboard thrust "<<endl;
        if(scanf("%lf",&input)){
            cout << input<<endl;
            //return acum+=0.02;
            return input;
        };

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
};





int main(int argc, char **argv)
{
    int x = 4;
    int sum = x * 3;
    cout << sum<<endl;

    ros::init(argc, argv, "offb");
    SonicFirmware sonic;
    ros::Rate rate(20.0);

    //the setpoint publishing rate MUST be faster than 2Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED_NOGPS";



    ros::Time last_request = ros::Time::now();
    mavros_msgs::CommandBool arm_cmd;

    while(ros::ok()){
        if( current_state.mode != "GUIDED_NOGPS" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( sonic.set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("GUIDED_NOGPS enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( /*(!current_state.armed &&)*/
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( sonic.arming_client.call(sonic.set_armed(&arm_cmd, true))  && !sonic.disarm){

                        ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(moving_status) /*(&& current_state.armed))*/ {
            if(current_state.mode == "GUIDED_NOGPS"){
                //sleep(5);
                //double thrust = sonic.thrust_keyboard();           //if you want to use thrust from keyboard
                sonic.forward(0.51,PI/2,2, rate);
                sonic.forward(0.7,0,2, rate);
                sonic.forward(0.4,0,2, rate);
                sonic.forward(0.1,0,2, rate);
                cout << "it called the function" <<endl;
                sonic.set_moving_status(false);
                sonic.set_armed(&arm_cmd, false);
            }

        }

    }

    return 0;
}