#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include <math.h>
#include "sonic/pointService.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#define STEP 20  // there are 40 steps in 1 meter (global planner)

using namespace std;

bool saveNewGlobalPlan = true;
nav_msgs::Path globalPath;
int globalPathSize;
int currentStep = 1;
visualization_msgs::MarkerArray markerArray;
geometry_msgs::Point goal;

ros::Publisher globalPlanPub;
ros::Publisher globalPlanMarkersPub;

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
    if(saveNewGlobalPlan) {
        globalPath.header = path.header;
        int oldPosesSize = path.poses.size();
        int newPosesSize = ceil(oldPosesSize / STEP);
        goal = path.poses[oldPosesSize-1].pose.position;

        int oldIndex = 0;
        for (int i = 0 ; i < newPosesSize ; i++) {
            globalPath.poses.push_back(path.poses[oldIndex]);
            markerArray.markers.push_back(createMarker(path.poses[oldIndex].pose.position, i));
            cout << "s" << i << ":" << endl << path.poses[oldIndex].pose.position << endl;

            oldIndex += STEP;
        }
        globalPath.poses.push_back(path.poses[oldPosesSize - 1]);
        markerArray.markers.push_back(createMarker(path.poses[oldPosesSize - 1].pose.position, newPosesSize));

        cout << "s" << newPosesSize << ":" << endl << path.poses[oldPosesSize - 1].pose.position << endl;

        globalPathSize = newPosesSize;
        saveNewGlobalPlan = false;
    }

    globalPlanPub.publish(globalPath);
    globalPlanMarkersPub.publish(markerArray);

    if(path.poses[path.poses.size()-1].pose.position != goal){
        currentStep = 1;
        saveNewGlobalPlan = true;
        globalPath.poses.clear();
    }
}


bool plannerService(sonic::pointService::Request &req, sonic::pointService::Response &res)
{
    if(req.ready){

        if (currentStep == globalPathSize + 1) {
            cout << "The goal has been reached, congratulations!" << endl;
            currentStep = 1;
            saveNewGlobalPlan = true;
            globalPath.poses.clear();
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
    // initialize ros
    ros::init(argc, argv, "sonic_planner");
    ros::NodeHandle n;

    // subscribe to global path planner
    ros::Subscriber globalPlannerSub = n.subscribe("move_base/DWAPlannerROS/global_plan", 1000, globalPlanner_cb);

    // service that gives next path goal
    ros::ServiceServer service = n.advertiseService("plannerService", plannerService);

    globalPlanPub = n.advertise<nav_msgs::Path>("new_global_plan", 10);
    globalPlanMarkersPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    ros::spin();

    return 0;
}