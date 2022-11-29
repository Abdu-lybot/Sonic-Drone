#! /usr/bin/env python

import time
import rospy
from std_srvs.srv import Trigger


while True:
    prox = rospy.ServiceProxy('rfid_readings_step_manager', Trigger)
    prox.call()
    time.sleep(2)



