#!/usr/bin/env python
import rospy
import traceback
#from self_exceptions import InvalidQueryDatabase
from database_robot.srv import DatabaseRobot, DatabaseRobotResponse
from lib.sources.database_operations import DatabaseOperations
from lib.sources.parameters import ParameterHelper
from lib.sources.response import ResponseHelper


class DatabaseController:
    def __init__(self):
        rospy.logdebug("[database_controller]: Starting service")
        #Init variables
        self.db_operation = DatabaseOperations()
        self.helper = ParameterHelper()
        #Init service
        rospy.Service('~database_controller', DatabaseRobot, self.cb_database_controller)
        rospy.logdebug("[database_controller]: Service started")

    def cb_database_controller(self, req):
        res = DatabaseRobotResponse()
        try:
            init_values, order = self.helper.obtain_init_values(req.table, req.mission, req.readings, req.shop,
                                                                req.section, req.fixture)
            if req.command == "INSERT":
                self.db_operation.insert(req.table, init_values, order)
                res.message = "OK"
            elif req.command == "SELECT":
                resp_helper = ResponseHelper()
                data = self.db_operation.select(req.table, init_values, req.field_list)
                resp_helper.parse_select(req.table, data, res, req.field_list)
            elif req.command == "SELECT_DISTINCT":
                resp_helper = ResponseHelper()
                data = self.db_operation.select_distinct(req.table, init_values, req.field_list)
                resp_helper.parse_select(req.table, data, res, req.field_list)
            elif req.command == "DELETE":
                self.db_operation.delete(req.table, init_values)
                res.message = "OK"
            elif req.command == "UPDATE":
                self.db_operation.update(req.table, init_values, req.field_list)
            else:
                res.status = False
                res.message = "[database_controller]: Command " + req.command + " not found"
                rospy.logerr(res.message)
                return res
        except:
            res.status = False
            #res.message = exc.message
            return res
        res.status = True
        return res


if __name__ == '__main__':
    try:
        rospy.init_node('database_robot', log_level=rospy.DEBUG)
        db = DatabaseController()
        rospy.spin()

    except Exception as e:
        rospy.logfatal('Exception %s', e.message + str(e.args))
        e = traceback.format_exc()
        rospy.logfatal(e)
