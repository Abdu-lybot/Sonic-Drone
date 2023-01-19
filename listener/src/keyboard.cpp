//
// Created by lybot on 7/07/22.
//
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <stdio.h>

using namespace std;

float key_throttle {1000}, key_pitch {0}, key_gripper {0};

double read_key() {
    cout << "Press any key to continue..." << endl;
    // Set terminal to raw mode
    //system("stty raw");
    // Wait for single character
    char input = getchar();
    switch (input) {
        case 'T':
            key_throttle += 10;
            printf(" throttle : %f\n", key_throttle);
            break;
        case 't':
            key_throttle -= 10;
            printf(" throttle : %f\n", key_throttle);
            break;
        case 'f':
            key_pitch += 1000;
            printf(" pitch : %f\n", key_pitch);
            break;
        case 'b':
            key_pitch -= 1000;
            printf(" pitch : %f\n", key_pitch);
            break;
        case 's':
            key_throttle = key_pitch  = key_gripper = 0;
            key_throttle = 1000;
            printf(" throttle : %f\n", key_throttle);
        break;
        case 'B':
            key_throttle = 1450;
            printf(" throttle : %f\n", key_throttle);
        break;
        case 'P':
            key_pitch += 50;
            printf(" pitch : %f\n", key_pitch);
        break;
        case 'p':
            key_pitch -= 50;
            printf(" pitch : %f\n", key_pitch);
        break;
        case 'G':
            key_gripper += 10;
            printf(" gripper : %f\n", key_gripper);
        break;
        case 'g':
            key_gripper -= 10;
            printf(" gripper : %f\n", key_gripper);
        break;
        case 'C':
            key_gripper = 1700;
            printf(" gripped closed, %f\n", key_gripper);
        break;
        case 'c':
            key_gripper = 1300;
            printf(" gripped opened, %f\n", key_gripper);
            break;
    }
    // Reset terminal to normal "cooked" mode
    //system("stty cooked");
    return 0;
}

geometry_msgs::Point create_msg(){
    geometry_msgs::Point msg;
    msg.x = key_throttle;
    msg.y = key_pitch;
    msg.z = key_gripper;
    return msg;
}
int main(int argc, char **argv){
    cout << "ENJOY SONIC" <<endl;
    ros::init(argc, argv, "Keyboard_input");
    ros::NodeHandle n;
    ros::Publisher pub_key = n.advertise<geometry_msgs::Point>("keyboard_sonic",1);

    while(ros::ok()){
        read_key();
        geometry_msgs::Point keyboard_command = create_msg();
        cout << keyboard_command <<endl;
        pub_key.publish(keyboard_command);

    }
}
