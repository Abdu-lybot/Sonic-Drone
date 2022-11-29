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

#define STEP 17   // there are 40 steps in 1 meter (global planner)

using namespace std;

bool saveNewGlobalPlan = true;
nav_msgs::Path globalPath;
int globalPathSize;
int currentStep = 0;

ros::Publisher globalPlanPub;

void globalPlanner(const nav_msgs::Path path)
{
    if(saveNewGlobalPlan) {
        globalPath.header = path.header;
        int oldPosesSize = path.poses.size();
        int newPosesSize = ceil(oldPosesSize / STEP);

        int oldIndex = 0;
        for (int i = 0 ; i < newPosesSize - 1 ; i++) {
            globalPath.poses.push_back(path.poses[oldIndex]);
            oldIndex += STEP;
        }
        globalPath.poses.push_back(path.poses[oldPosesSize - 1]);

        globalPathSize = newPosesSize;
        saveNewGlobalPlan = false;

    }

    globalPlanPub.publish(globalPath);

}


bool plannerService(listener::pointService::Request &req, listener::pointService::Response &res)
{
    if(req.ready){
        cout << "Ready to move, sending next point..." << endl;
        res.point = globalPath.poses.at(currentStep).pose.position;

        currentStep ++;
        if (currentStep == globalPathSize) {
            cout << "The goal has been reached, congratulations!" << endl;
            currentStep = 0;
            saveNewGlobalPlan = true;
        }
    } else {
        cout << "Drone not ready" << endl;
    }

    return true;
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    // subscribe to global path planner
    ros::Subscriber globalPlannerSub = n.subscribe("move_base/DWAPlannerROS/global_plan", 1000, globalPlanner);

    // service that gives next path goal
    ros::ServiceServer service = n.advertiseService("plannerService", plannerService);

    globalPlanPub = n.advertise<nav_msgs::Path>("new_global_plan", 10);


    ros::spin();

    return 0;
}