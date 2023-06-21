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

#define STEP 30  // there are 40 steps in 1 meter (global planner)
#define PI 3.14

using namespace std;

nav_msgs::Path globalPath;

int currentStep = 1;
visualization_msgs::MarkerArray markerArray;

ros::Publisher globalPlanPub;
ros::Publisher globalPlanMarkersPub;

geometry_msgs::Point initialPosition;

int markerStep = 0;

bool first = true;

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
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = 171.0/255.0;
    marker.color.g = 67.0/255.0;
    marker.color.b = 192.0/255.0;
    return marker;
}

void globalPlanner_cb(const nav_msgs::Path path)
{
    globalPath.header = path.header;
    int planner_size = path.poses.size();
    int new_size = ceil((float) planner_size / STEP);

    globalPath.poses.clear();
    markerArray.markers.clear();
    int old_step = 0;
    for (int i = 0 ; i < new_size ; i++) {
        globalPath.poses.push_back(path.poses[old_step]);
        markerArray.markers.push_back(createMarker(path.poses[old_step].pose.position, i));
        old_step += STEP;
    }
    if (old_step - planner_size < STEP/2){
        globalPath.poses.push_back(path.poses[planner_size - 1]);
        markerArray.markers.push_back(createMarker(path.poses[planner_size - 1].pose.position, new_size));
    }

    globalPlanPub.publish(globalPath);
    globalPlanMarkersPub.publish(markerArray);
}

void addStep(double x, double y) {
    geometry_msgs::Point point;
    point.x = x + initialPosition.x;
    point.y = y + initialPosition.y;
    geometry_msgs::Pose pose;
    pose.position = point;
    pose.orientation.w = 1;
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "camera_odom_frame";
    poseStamped.pose = pose;

    globalPath.poses.push_back(poseStamped);
    markerArray.markers.push_back(createMarker(point, markerStep));
    markerStep++;
}

void createSquare() {

    globalPath.poses.clear();
    markerArray.markers.clear();

    addStep(0,0);
    addStep(0.5,0);
//    addStep(1,0);
//    addStep(1.5,0);
//    addStep(1.5,0.5);
//    addStep(1.5,1);
//    addStep(0.6,0.6);
//    addStep(1,0);
//    addStep(1,0.5);
//    addStep(1,1);
//    addStep(0.5,1);
//    addStep(0,1);
//    addStep(0,0.5);
//    addStep(0,0);

    globalPath.header.frame_id = "camera_odom_frame";

    ros::Time last_request = ros::Time::now();
    while(ros::Time::now() - last_request < ros::Duration(5.0)){
        globalPlanPub.publish(globalPath);
        globalPlanMarkersPub.publish(markerArray);
    }
}

bool plannerService(listener::pointService::Request &req, listener::pointService::Response &res)
{
    if(req.ready){
//        if (first) {
//            initialPosition = req.initialPosition;
//            cout << "Initial position: " << initialPosition << endl;
//            createSquare();
//            first = false;
//        }
        if (currentStep >= globalPath.poses.size()) {
            cout << "The goal has been reached, congratulations!" << endl;
            currentStep = 1;
            return false;
        } else {
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
    ros::init(argc, argv, "Navigation");
    ros::NodeHandle n;

    // to get navigation plans from move_base
    ros::Subscriber globalPlannerSub = n.subscribe("move_base/DWAPlannerROS/global_plan", 1000, globalPlanner_cb);

    // for communication with the mobility module
    ros::ServiceServer service = n.advertiseService("plannerService", plannerService);

    // for visualization in RVIZ
    globalPlanPub = n.advertise<nav_msgs::Path>("new_global_plan", 10);
    globalPlanMarkersPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

//    createSquare();

    cout << "Navigation ready" << endl;

    ros::spin();

    return 0;
}