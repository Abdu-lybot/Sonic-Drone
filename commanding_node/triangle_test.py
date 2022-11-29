#!/usr/bin/env python

from triangle import MavController
import time


comm = MavController()

print("Extra time")
time.sleep(5)

comm.takeoff(0.5)

print("Extra time 2")
time.sleep(3)

#comm.goto_xyz_rpy(0,0,0.5,0,0,0)

#time.sleep(5)

#ctrl.goto_xyz_rpy(0,1,0.5,0,0,0)

#ctrl.goto_xyz_rpy(0,2,0.5,0,0,0)

#ctrl.goto_xyz_rpy(0,2,0.5,0,0,-1.57)

#ctrl.goto_xyz_rpy(1,2,0.5,0,0,-1.57)

#ctrl.goto_xyz_rpy(2,2,0.5,0,0,-1.57)

#ctrl.goto_xyz_rpy(2,2,0.5,0,0,-3.14)

#ctrl.goto_xyz_rpy(2,1,0.5,0,0,-3.14)

#ctrl.goto_xyz_rpy(2,0,0.5,0,0,-3.14)


comm.change_pos(0,0,0.5)

comm.change_pos(0,0.5,0.5)

comm.change_pos(0,1,0.5)

comm.change_pos(0,1.5,0.5)

comm.change_pos(0,2,0.5)

comm.change_pos(0,2.5,0.5)

comm.change_pos(0,3,0.5)

comm.change_ori(-1.57)

#comm.change_pos(0.5,2,0.5)

#comm.change_pos(1,2,0.5)

#comm.change_pos(1.5,2,0.5)

#comm.change_pos(2,2,0.5)

#comm.change_ori(-3.14)

#comm.change_pos(2,1.5,0.5)

#comm.change_pos(2,1,0.5)

#comm.change_pos(2,0.5,0.5)

#comm.change_pos(2,0,0.5)

#comm.change_ori(1.57)

#comm.change_pos(1.5,0,0.5)

#comm.change_pos(1,0,0.5)

#comm.change_pos(0.5,0,0.5)

#comm.change_pos(0,0,0.5)

#comm.change_ori(0)

comm.land()
