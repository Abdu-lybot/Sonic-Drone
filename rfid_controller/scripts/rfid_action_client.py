#! /usr/bin/env python

import rospy
import actionlib
from rfid_controller.msg import RFIDControllerAction, RFIDControllerGoal


def rfid_client():
    client = actionlib.SimpleActionClient('/rfid_controller', RFIDControllerAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server(timeout=rospy.Duration(10.0))

    # Creates a goal to send to the action server.
    goal = RFIDControllerGoal(mission_id="prova", section_id="C1", task="inventory")

    # Sends the goal to the action server.
    client.send_goal(goal)

    #Wait some time
    rospy.sleep(10)

    #Cancel action
    client.cancel_all_goals()

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('rfid_client_py')
        result = rfid_client()
        print("Result: " + str(result.success))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
