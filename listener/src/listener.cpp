#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include <math.h>
#include "listener/pointService.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <fstream>
#include <thread>

#define STEP 20  // there are 40 steps in 1 meter (global planner)
#define PI 3.14

using namespace std;

std::ifstream goals_file("src/listener/src/square.txt");

bool use_file_for_goals = false;

bool saveNewGlobalPlan = true;
nav_msgs::Path globalPath;
int globalPathSize;

nav_msgs::Path aux_globalPath;
int aux_globalPathSize;

int currentStep = 1;
visualization_msgs::MarkerArray markerArray;
geometry_msgs::Point goal;

visualization_msgs::MarkerArray aux_markerArray;

std::vector<geometry_msgs::PoseStamped> file_goals;
int current_file_goal = 0;
bool save_file_goals = true;

ros::Publisher globalPlanPub;
ros::Publisher globalPlanMarkersPub;
ros::Publisher move_base_simple_goal_pub;

float vslam_x, vslam_y, vslam_z;

visualization_msgs::Marker createMarker(geometry_msgs::Point p, int s)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_odom_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "sonic";
    marker.id = s;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "s" + to_string(s);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 171.0/255.0;
    marker.color.g = 67.0/255.0;
    marker.color.b = 192.0/255.0;
//only if using a MESH_RESOURCE marker type:
//    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    return marker;
}

void globalPlanner_cb(const nav_msgs::Path path)
{

//    if(saveNewGlobalPlan) {
        globalPath.header = path.header;
        int oldPosesSize = path.poses.size();
        int newPosesSize = ceil((float) oldPosesSize / STEP);
//        goal = path.poses[oldPosesSize-1].pose.position;

        aux_globalPath.poses.clear();
        int oldIndex = 0;
        for (int i = 0 ; i < newPosesSize ; i++) {
            aux_globalPath.poses.push_back(path.poses[oldIndex]);
            aux_markerArray.markers.push_back(createMarker(path.poses[oldIndex].pose.position, i));
//            cout << "s" << i << ":" << endl << path.poses[oldIndex].pose.position << endl;

            oldIndex += STEP;
        }
//        globalPath.poses.push_back(path.poses[oldPosesSize - 1]);
//        markerArray.markers.push_back(createMarker(path.poses[oldPosesSize - 1].pose.position, newPosesSize));

//        cout << "s" << newPosesSize << ":" << endl << path.poses[oldPosesSize - 1].pose.position << endl;

//        aux_globalPathSize = newPosesSize;
//        saveNewGlobalPlan = false;
//    }

    if (saveNewGlobalPlan) {
        globalPath.poses.clear();
        markerArray.markers.clear();
//        globalPathSize = aux_globalPathSize;
        for (int i = 0; i < aux_globalPath.poses.size(); ++i) {
            globalPath.poses.push_back(aux_globalPath.poses[i]);
            markerArray.markers.push_back(aux_markerArray.markers[i]);
        }
//        aux_globalPath.poses.clear();
        saveNewGlobalPlan = true;
    }

    globalPlanPub.publish(globalPath);
    globalPlanMarkersPub.publish(markerArray);

//    if(path.poses[path.poses.size()-1].pose.position != goal){
//        currentStep = 1;
//        saveNewGlobalPlan = true;
//        globalPath.poses.clear();
//    }
}

void setGoalsUsingFile() {
    int i = 0;
    float a, b;
    while (goals_file >> a >> b)
    {
        cout << a << " " << b << endl;
        geometry_msgs::Point point;
        point.x = a;
        point.y = b;
        geometry_msgs::Pose pose;
        pose.position = point;
        pose.orientation.w = 1;
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "camera_odom_frame";
        poseStamped.pose = pose;
        file_goals.push_back(poseStamped);
        globalPath.poses.push_back(poseStamped);
        markerArray.markers.push_back(createMarker(point, i));
        i ++;
    }

//    file_goals[current_file_goal].pose.position.x += vslam_x;
//    file_goals[current_file_goal].pose.position.y += vslam_y;
    ros::Time last_request = ros::Time::now();
    ROS_INFO("Publishing first goal");
    while((ros::Time::now() - last_request < ros::Duration(1))) {
        move_base_simple_goal_pub.publish(file_goals[current_file_goal]);
    }
    cout << file_goals[current_file_goal] << endl;
    current_file_goal ++;

    cout << "file_goals size: " << file_goals.size() << endl;

//    globalPathSize = globalPath.poses.size();

}

//float get_distance(geometry_msgs::Point goal){
//    float d = sqrt(pow(goal.x - vslam_x,2) + pow(goal.y - vslam_y,2));
//    return d;
//}
//bool in_margin_dist(geometry_msgs::Point goal){
//
//    if (get_distance(goal) <  0.15) {
//        ROS_ERROR("Goal Reached");
//        return true;
//    }
//    else{
//        ROS_ERROR("Rolling Forward, distance left: %f", get_distance(goal));
//        return false;
//    }
//}


void vslam_cb(nav_msgs::Odometry data){
    vslam_x = data.pose.pose.position.x;
    vslam_y = data.pose.pose.position.y;
    vslam_z = data.pose.pose.position.z;
    double roll, pitch, yaw;
    float vslam_ori_x = data.pose.pose.orientation.x;
    float vslam_ori_y = data.pose.pose.orientation.y;
    float vslam_ori_z = data.pose.pose.orientation.z;
    float vslam_ori_w = data.pose.pose.orientation.w;
    tf::Quaternion q(vslam_ori_x, vslam_ori_y, vslam_ori_z, vslam_ori_w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    float yaw_degrees = yaw * 180.0 / PI;
//    cout << "current: "<< yaw_degrees << endl;
//    double yaw_diff = -150 - yaw_degrees;
//    cout << "yaw diff: " << yaw_diff << endl;
//    if (yaw_diff > 180) yaw_diff -= 360;
//    if (yaw_diff <= -180) yaw_diff += 360;
//    if (yaw_diff > 0){
//        cout << "ccw" << endl; //positive
//    } else {
//        cout << "cw" << endl; //negative
//    }


//    if (save_file_goals && use_file_for_goals) {
//        save_file_goals = false;
//        setGoalsUsingFile();
//    }

//    if(in_margin_dist(file_goals[current_file_goal].pose.position)) {
//        cout << "GOAL REACHED" << endl;
//        ros::Time last_request = ros::Time::now();
//        current_file_goal++;
//        while((ros::Time::now() - last_request < ros::Duration(0.5))) {
//            ROS_INFO("Publishing next goal");
//            move_base_simple_goal_pub.publish(file_goals[current_file_goal]);
//        }
//    }
}

bool plannerService(listener::pointService::Request &req, listener::pointService::Response &res)
{
    if(req.ready){
        if (currentStep >= globalPath.poses.size()) {
            cout << "The goal has been reached, congratulations!" << endl;
            currentStep = 1;
            saveNewGlobalPlan = true;
            return false;
            if(file_goals.size() == current_file_goal) return false;
//            file_goals[current_file_goal].pose.position.x += vslam_x;
//            file_goals[current_file_goal].pose.position.y += vslam_y;
//            ros::Time last_request = ros::Time::now();
//            ROS_INFO("Publishing next goal");
//            cout << "Publishing goal nº: " << current_file_goal << endl;
//            while((ros::Time::now() - last_request < ros::Duration(1))) {
//                move_base_simple_goal_pub.publish(file_goals[current_file_goal]);
//            }
            current_file_goal ++;
//            res.again = true;
            globalPath.poses.clear();
            globalPathSize = aux_globalPathSize;
            for (int i = 0; i < globalPathSize; ++i) {
                globalPath.poses.push_back(aux_globalPath.poses[i]);
                markerArray.markers.push_back(aux_markerArray.markers[i]);
            }
            res.point = globalPath.poses.at(currentStep).pose.position;
            cout << "Ready to move, next goal s" << currentStep << ":" << endl << res.point << endl;
        } else {
//            res.again = false;
            res.point = globalPath.poses.at(currentStep).pose.position;
            cout << "Ready to move, next goal s" << currentStep << ":" << endl << res.point << endl;
        }
        currentStep ++;

    } else {
        cout << "Drone not ready" << endl;
    }

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber globalPlannerSub = n.subscribe("move_base/DWAPlannerROS/global_plan", 1000, globalPlanner_cb);

    ros::Subscriber vslam_pose = n.subscribe<nav_msgs::Odometry>("camera/odom/sample", 10, vslam_cb);

    ros::ServiceServer service = n.advertiseService("plannerService", plannerService);

    globalPlanPub = n.advertise<nav_msgs::Path>("new_global_plan", 10);
    globalPlanMarkersPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    move_base_simple_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    if (use_file_for_goals) {
        setGoalsUsingFile();
    }

    cout << "listener ready" << endl;

    ros::spin();

    return 0;
}