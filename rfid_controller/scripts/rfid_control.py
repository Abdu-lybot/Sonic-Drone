#!/usr/bin/python
import rospy
import traceback
import actionlib
from rfid_controller.msg import RFIDControllerAction, RFIDControllerFeedback, RFIDControllerResult
from lib.sources.readers import Readers
from lib.sources.readings import ReadingQueue, ManageReadings


class RFIDController:
    '''
    Class that controls the RFID of the robot
    '''
    def __init__(self, name):
        #Init variables
        self.mission_id = None
        self.section_id = None
        self.task = None
        self.readers = None
        self.reading_queue = ReadingQueue()
        self.manage_readings = None
        #Start action
        self._action_name = name
        self._feedback = RFIDControllerAction()
        self._result = RFIDControllerResult()
        self._as = actionlib.SimpleActionServer(self._action_name, RFIDControllerAction,
                                                execute_cb=self.start_rfid_cb, auto_start=False)
        self._as.register_preempt_callback(self.cancel_rfid_cb)
        self._as.start()

    #RFID action callbacks
    def cancel_rfid_cb(self):
        '''
        Callback reaction to a cancel request
        :return:
        '''
        rospy.loginfo('[RFIDContoller]: RFID has been cancelled')
        self.readers.stop()
        self.manage_readings.close()

    def start_rfid_cb(self, request):
        '''
        Callback reaction to a new goal request
        :param request:
        :return:
        '''
        #Obtain request parameters
        self.mission_id = request.mission_id
        self.section_id = request.section_id
        self.task = request.task
        rospy.loginfo("[RFIDContoller]: Starting action mission %s, section %s and task %s", self.mission_id,
                      self.section_id, self.task)
        #Init variables
        self.manage_readings = ManageReadings(self.reading_queue, self.mission_id, self.section_id)
        self.readers = Readers(self.reading_queue)
        #Set-up readers
        is_set_up = self.readers.set_up(self.task)
        if not is_set_up:
            #return error
            pass
        #Start readers (blocking call)
        res = self.readers.start()
        #Send result
        self._result.success = res
        rospy.loginfo('[RFIDContoller]: finished with success = %s', str(res))
        if res:
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted(self._result)


if __name__ == '__main__':
    rospy.init_node('rfid_controller', log_level=rospy.DEBUG)
    try:
        RFIDController(rospy.get_name())
        rospy.spin()
        
    except Exception as e:
        rospy.logfatal('[RFIDController] RFID controller died')
        e = traceback.format_exc()
        rospy.logfatal(e)
