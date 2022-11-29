#!/usr/bin/env python
import rospy
from database_robot.msg import Mission, Reading, Shop, Section, Fixture


class ResponseHelper:
    def __init__(self):
        self.order_mission = rospy.get_param('database_robot/database/order_mission')
        self.order_shop = rospy.get_param('database_robot/database/order_shop')
        self.order_section = rospy.get_param('database_robot/database/order_section')
        self.order_reading = rospy.get_param('database_robot/database/order_reading')
        self.order_fixture = rospy.get_param('database_robot/database/order_fixture')

    def list_to_mission(self, data, field_list):
        res = list()
        #Create first row
        message = str(self.order_mission)
        for row in data:
            mission = Mission()
            mission.id_mission = str(row[0])
            mission.start = str(row[1])
            mission.finish = str(row[2])
            mission.sections = str(row[3])
            mission.task = str(row[4])
            mission.processed = str(row[5])
            res.append(mission)
        return message, res

    def list_to_readings(self, data,field_list):
        res = list()
        #Create first row
        if field_list:
            message = field_list
        else:
            message = self.order_reading
        for row in data:
            reading = Reading()
            for i in range(0, len(message)):
                if message[i] == "id_reading":
                    reading.id_reading = str(row[i])
                elif message[i] == "id_mission":
                    reading.id_mission = str(row[i])
                elif message[i] == "id_section":
                    reading.id_section = str(row[i])
                elif message[i] == "antenna_port":
                    reading.antenna_port = str(row[i])
                elif message[i] == "timestamp":
                    reading.timestamp = str(row[i])
                elif message[i] == "rf_phase":
                    reading.rf_phase = str(row[i])
                elif message[i] == "rssi":
                    reading.rssi = str(row[i])
                elif message[i] == "frq":
                    reading.freq = str(row[i])
                elif message[i] == "mux1":
                    reading.mux1 = str(row[i])
                elif message[i] == "mux2":
                    reading.mux2 = str(row[i])
                elif message[i] == "epc":
                    reading.epc = row[i]
                elif message[i] == "device_id":
                    reading.device_id = row[i]
                elif message[i] == "robot_pose":
                    reading.robot_pose = row[i]
            res.append(reading)
        return str(message), res

    def list_to_shop(self, data, field_list):
        res = list()
        # Create first row
        message = str(self.order_shop)
        for row in data:
            shop = Shop()
            shop.id_shop = str(row[0])
            shop.name = str(row[1])
            shop.description = str(row[2])
            res.append(shop)
        return message, res

    def list_to_section(self, data, field_list):
        res = list()
        #Create first row
        if field_list:
            message = field_list
        else:
            message = self.order_section
        for row in data:
            section = Section()
            for i in range(0, len(message)):
                if message[i] == "id_section":
                    section.id_section = str(row[i])
                elif message[i] == "name":
                    section.name = str(row[i])
                elif message[i] == "qr_docking":
                    section.qr_docking = str(row[i])
                elif message[i] == "maps_path":
                    section.maps_path = str(row[i])
                elif message[i] == "description":
                    section.description = str(row[i])
                elif message[i] == "id_shop":
                    section.id_shop = str(row[i])
                elif message[i] == "prev_section":
                    section.prev_section = str(row[i])
            res.append(section)
        return str(message), res

    def list_to_fixture(self, data, field_list):
        res = list()
        #Create first row
        if field_list:
            message = field_list
        else:
            message = self.order_fixture
        for row in data:
            fixture = Fixture()
            for i in range(0, len(message)):
                if message[i] == "id_fixture":
                    fixture.id_fixture = str(row[i])
                elif message[i] == "id_section":
                    fixture.id_section = str(row[i])
                elif message[i] == "area":
                    fixture.area = str(row[i])
                elif message[i] == "fix_group":
                    fixture.fix_group = str(row[i])
                elif message[i] == "ref_tag":
                    fixture.ref_tag = str(row[i])
                elif message[i] == "x":
                    fixture.x = str(row[i])
                elif message[i] == "y":
                    fixture.y = str(row[i])
                elif message[i] == "z":
                    fixture.z = str(row[i])
            res.append(fixture)
        return str(message), res

    def parse_select(self, table, data, response, field_list):
        if table == "Missions":
            response.message, response.missions = self.list_to_mission(data, field_list)
        elif table == "Readings":
            response.message, response.readings = self.list_to_readings(data, field_list)
        elif table == "Shops":
            response.message, response.shops = self.list_to_shop(data, field_list)
        elif table == "Sections":
            response.message, response.sections = self.list_to_section(data, field_list)
        elif table == "Fixtures":
            response.message, response.fixtures = self.list_to_fixture(data, field_list)
        return response
