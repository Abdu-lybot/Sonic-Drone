#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include "listener/pointService.h"

#include <sstream>

using namespace std;


void instructionsCallback(const std_msgs::Float64MultiArray msg)
{
    float rotation = msg.data[0];
    float forward = msg.data[1];

    std::cout << "Rotation: " << rotation << std::endl;
    std::cout << "Forward: " << forward << std::endl;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;


//    ros::Subscriber sub = n.subscribe("instructions", 1000, instructionsCallback);
//    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


    ros::ServiceClient client = n.serviceClient<listener::pointService>("plannerService");
    listener::pointService srv;
    srv.request.ready = true;
    ros::service::waitForService("plannerService", ros::Duration(5));
    while (true) {
        if (client.call(srv))
        {
            cout << "Next goal: " << endl << srv.response.point << endl;
        }
        else
        {
            cout << "ERROR" << endl;
        }
        sleep(5);
    }



//    int count = 0;
//    while (ros::ok())
//    {
//
//
//        std_msgs::String msg;
//
//        std::string s;
//        std::cin >> s;
//
//        msg.data = s;
//
//
//
//        chatter_pub.publish(msg);
//
//        ros::spinOnce();
//
//        loop_rate.sleep();
//        ++count;
//
//
//
//
//    }


    return 0;
}