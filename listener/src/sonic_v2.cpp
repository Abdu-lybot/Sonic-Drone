//
// Created by lybot on 13/4/22.
//

//#include "sonic_v2.h"
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
#define GOAL_POSITION_MARGIN 0.15
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
    float steering_left = 1225;
    float steering_forward = 1555;
    float steering_right = 1905;
    float brake_left = 1700;
    float brake_neutral = 1180;
    float brake_right = 2000;
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
        if ((yaw_diff > 0 && yaw_diff <= 180) || (yaw_diff < 0 && yaw_diff >= 180)){
            ROS_INFO_NAMED("Turn"," CCW is closer to goal");
            return "ccw";
        }else{
            ROS_INFO_NAMED("Turn"," CW is closer to goal");
            return "cw";
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
        manual_move(0, 0, 0, 000, 0, brake_neutral, brake_neutral, steering_forward,0);
    }

    void turnccw(double angle){
        ROS_INFO("Sonic is moving left. Goal: %lf. Current: %lf.", angle, yaw_degrees);
        manual_move(0, 0, 0, 000, 0, brake_neutral, brake_left, steering_left,0);
    }

    void turncw(double angle){
        ROS_INFO("Sonic is moving right. Goal: %lf. Current: %lf.", angle, yaw_degrees);
        manual_move(0, 0, 0, 000, 0, brake_right, brake_neutral, steering_right,0);
    }

    void roll_turn(float angle, ros::Rate rate){
        double yaw_diff = angle - yaw_degrees;
        while (in_margin_angle(yaw_diff)){
            yaw_diff = angle - yaw_degrees;
            if (cw_or_ccw(yaw_diff) == "ccw"){
                turnccw(angle);
                rate.sleep();
                ros::spinOnce();
            }
            if(cw_or_ccw(yaw_diff) == "cw"){
                turncw(angle);
                rate.sleep();
                ros::spinOnce();
            }
        }
    }

    void roll_forw(geometry_msgs::Point goal, ros::Rate rate){
        while(!in_margin_dist(goal)){
            forward(goal);
            rate.sleep();
            ros::spinOnce();
        }
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
                manual_move(0, 0, 0, 000, 0, brake_right, brake_left, steering_forward,0);
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

void move_forward(SonicFirmware sonic, ros::Rate rate, float distance){
    int PITCH = 1895;
    int THROTTLE = 1050;
    int STOP_THROTTLE = 1000;

    geometry_msgs::Point initialPosition;
    initialPosition.x = vslam_x;
    initialPosition.y = vslam_y;

    //Moverse
    float distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
    ros::Time last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(5.0)) && moving_status && distance_moved < (distance / 2)) { //0.25
        cout << "Throttle : "<<THROTTLE<<", Pitch : "<<PITCH<<", Gripper : "<< key_gripper << endl;
        sonic.manual_move(0, PITCH, THROTTLE, 0, 0, 1500, 1200, sonic.steering_forward,0); //forward
        rate.sleep();
        ros::spinOnce();
        distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
        cout << distance_moved << endl;
        THROTTLE = THROTTLE < 1025 ? 1025 : THROTTLE - 1;
    }

    //Stop thrust
    last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(1)) && moving_status && distance_moved < (distance - 0.10)) {
        sonic.manual_move(0, PITCH, STOP_THROTTLE, 0, 0, 1500, 1200, 0,0); //forward
        distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
        rate.sleep();
        ros::spinOnce();
        cout << "Sin Throttle" << endl;
    }

    //Freno
    last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(2.0)) && moving_status) {
        sonic.manual_move(0, PITCH, STOP_THROTTLE, 0, 0, sonic.brake_right, sonic.brake_left, sonic.steering_forward,0);
        rate.sleep();
        ros::spinOnce();
        cout << "Freno" << endl;
    }
}

void turn_cw(SonicFirmware sonic, ros::Rate rate, float distance){
    int PITCH = 1895;
    int THROTTLE = 1050;
    int STOP_THROTTLE = 1000;

    geometry_msgs::Point initialPosition;
    initialPosition.x = vslam_x;
    initialPosition.y = vslam_y;

    //Moverse
    float distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
    ros::Time last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(8.0)) && moving_status && distance_moved < (distance / 2)) { //0.25
        cout << "Throttle : "<<THROTTLE<<", Pitch : "<<PITCH<<", Gripper : "<< key_gripper << endl;
        sonic.manual_move(0, PITCH, THROTTLE, 1650, 0, sonic.brake_right, 1200, sonic.steering_right,0); //turn right
        rate.sleep();
        ros::spinOnce();
        distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
        cout << distance_moved << endl;
        THROTTLE = THROTTLE < 1025 ? 1025 : THROTTLE - 1;
    }

    //Stop thrust
    last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(1)) && moving_status && distance_moved < (distance - 0.10)) {
        sonic.manual_move(0, PITCH, STOP_THROTTLE, 0, 0, sonic.brake_right, 1200, sonic.steering_right,0); //turn right
        distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
        rate.sleep();
        ros::spinOnce();
        cout << "Sin Throttle" << endl;
    }

    //Freno
    last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(2.0)) && moving_status) {
        sonic.manual_move(0, PITCH, STOP_THROTTLE, 0, 0, sonic.brake_right, sonic.brake_left, sonic.steering_forward,0);
        rate.sleep();
        ros::spinOnce();
        cout << "Freno" << endl;
    }
}

void turn_ccw(SonicFirmware sonic, ros::Rate rate, float distance){
    int PITCH = 1895;
    int THROTTLE = 1050;
    int STOP_THROTTLE = 1000;

    geometry_msgs::Point initialPosition;
    initialPosition.x = vslam_x;
    initialPosition.y = vslam_y;

    //Moverse
    float distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
    ros::Time last_request = ros::Time::now();
    bool flag = true;
    while((ros::Time::now() - last_request < ros::Duration(8.0)) && moving_status && distance_moved < (distance / 2)) { //0.25
        cout << "Throttle : "<<THROTTLE<<", Pitch : "<<PITCH<<", Gripper : "<< key_gripper << endl;
        sonic.manual_move(0, PITCH, THROTTLE, 1100, 0, 1500, sonic.brake_left, sonic.steering_left,0); //turn left
        rate.sleep();
        ros::spinOnce();
        distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
        cout << distance_moved << endl;
        THROTTLE = THROTTLE < 1025 ? 1025 : THROTTLE - 1;
    }

    //Stop thrust
    last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(1)) && moving_status && distance_moved < (distance - 0.10)) {
        sonic.manual_move(0, PITCH, STOP_THROTTLE, 1100, 0, 1500, sonic.brake_left, sonic.steering_left,0); //turn left
        distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));
        rate.sleep();
        ros::spinOnce();
        cout << "Sin Throttle" << endl;
    }

    //Freno
    last_request = ros::Time::now();
    while((ros::Time::now() - last_request < ros::Duration(2.0)) && moving_status) {
        sonic.manual_move(0, PITCH, STOP_THROTTLE, 0, 0, sonic.brake_right, sonic.brake_left, sonic.steering_forward,0);
        rate.sleep();
        ros::spinOnce();
        cout << "Freno" << endl;
    }
}

double getAngleValue(double angle) {
    if (angle == 90) {
        return 0.7;
    } else if (angle == 45) {
        return 0.4;
    } else {
        // calculate the value based on the linear relationship
        double slope = (0.7 - 0.4) / (90 - 45);
        double intercept = 0.4 - slope * 45;
        return slope * angle + intercept;
    }
}

double getTurnAngle(geometry_msgs::Point goal) {
    double dif_x = goal.x - vslam_x;
    double dif_y = goal.y - vslam_y;
    double targetAngle = atan2(dif_y, dif_x) * 180.0 / PI;
    double turnAngle = targetAngle - yaw_degrees;
    while (turnAngle > 180.0) {
        turnAngle -= 360.0;
    }
    while (turnAngle < -180.0) {
        turnAngle += 360.0;
    }
    return turnAngle;
}

// not working (sometimes I need the cosine, sometimes the sine)
double getForwardDistance(geometry_msgs::Point goal) {
    double dif_x = goal.x - vslam_x;
    double dif_y = goal.y - vslam_y;
    double distance = sqrt(dif_x * dif_x + dif_y * dif_y);
    double angle = abs(getTurnAngle(goal));
    return distance * cos(angle * PI / 180.0);
}

double getDistance(geometry_msgs::Point goal) {
    double dif_x = goal.x - vslam_x;
    double dif_y = goal.y - vslam_y;
    return sqrt(dif_x * dif_x + dif_y * dif_y);
}

bool shouldGoForward(geometry_msgs::Point goal) {
    geometry_msgs::Point pointAhead;
    pointAhead.x = vslam_x + 0.20 * cos(yaw);
    pointAhead.y = vslam_y + 0.20 * sin(yaw);
    return getDistance(goal) > getDistance(pointAhead);
}

int main(int argc, char **argv)
{
    cout << "SOnic V2 Library "<<endl;
    ros::init(argc, argv, "Sonic_Drone");
    //flightmode = "ACRO";  //STABILIZE
    flightmode = "STABILIZE";  //STABILIZE

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
            ros::Time last_request = ros::Time::now();

            last_request = ros::Time::now();
            while((ros::Time::now() - last_request < ros::Duration(5.0)) && moving_status) {
                sonic.manual_move(0, 0, 0, 0, 0, 1800, 1500, sonic.steering_forward,0);
                rate.sleep();
                ros::spinOnce();
                cout << "Encendiendo" << endl;
            }

            geometry_msgs::Point initialPosition;
            initialPosition.x = vslam_x;
            initialPosition.y = vslam_y;

            ros::Time startTime = ros::Time::now();

            sonic.srv.request.ready = true;
            sonic.srv.request.initialPosition = initialPosition;
            ros::service::waitForService("plannerService", ros::Duration(5));
            int precaution = 10;
            while (sonic.client.call(sonic.srv) && precaution > 0) {
                precaution --;
                geometry_msgs::Point goal = sonic.srv.response.point;
                cout << "GOAL: " << goal << endl;

                double turnAngle = getTurnAngle(goal);
                double value = getAngleValue(abs(turnAngle));

                if (abs(turnAngle) < 10) {
                    cout << "Angle in margin" << endl;
                } else if (turnAngle > 0) {
                    cout << "Turn ccw " << turnAngle << " degrees\n" << value << " opposite distance\n";
                    turn_ccw(sonic, rate, value);
                } else if (turnAngle < 0) {
                    cout << "Turn cw " << -turnAngle << " degrees\n" << value << " opposite distance\n";
                    turn_cw(sonic, rate, value);
                }
                double distance = getDistance(goal);
                if (shouldGoForward(goal) && distance < 1) {
                    cout << "Moving forward: " << distance << endl;
                    move_forward(sonic, rate, distance);
                } else {
                    cout << "Distance in margin: " << distance << endl;
                }
            }

            ros::Time endTime = ros::Time::now();


            float distance_moved = sqrt(pow(initialPosition.x - vslam_x, 2) + pow(initialPosition.y - vslam_y, 2));

            //Reposo
            last_request = ros::Time::now();
            while((ros::Time::now() - last_request < ros::Duration(4.0)) && moving_status) {
                sonic.manual_move(0, 0, 0, 0, 0, 1800, 1500, 0,0);
                rate.sleep();
                ros::spinOnce();
                cout << "Apagando" << endl;
            }

            cout << "distance moved: " << distance_moved << endl;
            cout << "time: " << endTime - startTime << endl;

            ROS_WARN("Function Ended");
            sonic.set_moving_status(false);
            //sonic.set_armed(&arm_cmd, false);
            ros::spin();
        }
    }
    return 0;
}