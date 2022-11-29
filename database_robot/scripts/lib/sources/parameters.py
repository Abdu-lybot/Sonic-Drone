#!/usr/bin/env python
import rospy
#from self_exceptions import InvalidParameterDatabase


class ParameterHelper:
    def __init__(self):
        self.order_mission = rospy.get_param('database_robot/database/order_mission')
        self.order_shop = rospy.get_param('database_robot/database/order_shop')
        self.order_section = rospy.get_param('database_robot/database/order_section')
        self.order_reading = rospy.get_param('database_robot/database/order_reading')
        self.order_fixture = rospy.get_param('database_robot/database/order_fixture')

    def _parameter_exist(self, parameter):
        '''
        Check if a parameter exists, parameter is always a string (as given by the message)
        :param parameter: parameter to check
        :return: True if there is a value, False otheriwse
        '''
        if not parameter or parameter == 'None':
            return False
        else:
            return True

    def mission_to_dict_list(self, values):
        '''
        Convert a Mission.msg into a list of dictionaries
        :param values: Mission.msg
        :return: the list of disctionaries plus the order of the fields in the DB
        '''
        res = list()
        mission = dict()
        if self._parameter_exist(values.id_mission):
            mission["id_mission"] = values.id_mission
        if self._parameter_exist(values.start):
            mission["start"] = values.start
        if self._parameter_exist(values.finish):
            mission["finish"] = values.finish
        if self._parameter_exist(values.sections):
            mission["sections"] = values.sections
        if self._parameter_exist(values.task):
            mission["task"] = values.task
        if self._parameter_exist(values.processed):
            mission["processed"] = values.processed == "True"
        if mission:
            res.append(mission)
        return res, self.order_mission

    def shop_to_dict_list(self, values):
        '''
        Convert a Shop.msg into a list of dictionaries
        :param values: Shop.msg
        :return: the list of disctionaries plus the order of the fields in the DB
        '''
        res = list()
        shop = dict()
        if self._parameter_exist(values.id_shop):
            shop["id_shop"] = int(values.id_shop)
        if self._parameter_exist(values.name):
            shop["name"] = values.name
        if self._parameter_exist(values.description):
            shop["description"] = values.description
        if shop:
            res.append(shop)
        return res, self.order_shop

    def section_to_dict_list(self, values):
        '''
        Convert a Section.msg into a list of dictionaries
        :param values: Section.msg
        :return: the list of disctionaries plus the order of the fields in the DB
        '''
        res = list()
        section = dict()
        if self._parameter_exist(values.id_section):
            section["id_section"] = values.id_section
        if self._parameter_exist(values.name):
            section["name"] = values.name
        if self._parameter_exist(values.qr_docking):
            section["qr_docking"] = values.qr_docking
        if self._parameter_exist(values.maps_path):
            section["maps_path"] = values.maps_path
        if self._parameter_exist(values.description):
            section["description"] = values.description
        if self._parameter_exist(values.id_shop):
            section["id_shop"] = int(values.id_shop)
        if self._parameter_exist(values.prev_section):
            section["prev_section"] = values.prev_section
        if section:
            res.append(section)
        return res, self.order_section

    def readings_to_dict_list(self, values):
        '''
        Convert a list of Readings.msg into a list of dictionaries
        :param values: Readings.msg
        :return: the list of disctionaries plus the order of the fields in the DB
        '''
        res = list()
        for value in values:
            reading = dict()
            if self._parameter_exist(value.id_reading):
                reading["id_reading"] = value.id_reading
            if self._parameter_exist(value.id_mission):
                reading["id_mission"] = value.id_mission
            if self._parameter_exist(value.id_section):
                reading["id_section"] = value.id_section
            if self._parameter_exist(value.antenna_port):
                reading["antenna_port"] = int(value.antenna_port)
            if self._parameter_exist(value.timestamp):
                reading["timestamp"] = int(value.timestamp)
            if self._parameter_exist(value.rf_phase):
                reading["rf_phase"] = int(value.rf_phase)
            if self._parameter_exist(value.rssi):
                reading["rssi"] = int(value.rssi)
            if self._parameter_exist(value.freq):
                reading["freq"] = int(value.freq)
            if self._parameter_exist(value.mux1):
                reading["mux1"] = int(value.mux1)
            if self._parameter_exist(value.mux2):
                reading["mux2"] = int(value.mux2)
            if self._parameter_exist(value.epc):
                reading["epc"] = value.epc
            if self._parameter_exist(value.device_id):
                reading["device_id"] = value.device_id
            if self._parameter_exist(value.robot_pose):
                reading["robot_pose"] = value.robot_pose
            if reading:
                res.append(reading)
        return res, self.order_reading

    def fixture_to_dict_list(self, values):
        '''
        Convert a Fixture.msg into a list of dictionaries
        :param values: Fixture.msg
        :return: the list of disctionaries plus the order of the fields in the DB
        '''
        res = list()
        fixture = dict()
        if self._parameter_exist(values.id_fixture):
            fixture["id_fixture"] = values.id_fixture
        if self._parameter_exist(values.id_shop):
            fixture["id_shop"] = int(values.id_shop)
        if self._parameter_exist(values.id_section):
            fixture["id_section"] = values.id_section
        if self._parameter_exist(values.name):
            fixture["name"] = values.name
        if self._parameter_exist(values.area):
            fixture["area"] = values.area
        if self._parameter_exist(values.fix_group):
            fixture["fix_group"] = values.fix_group
        if self._parameter_exist(values.ref_tag):
            fixture["ref_tag"] = values.ref_tag
        if self._parameter_exist(values.x):
            fixture["x"] = float(values.x)
        if self._parameter_exist(values.y):
            fixture["y"] = float(values.y)
        if self._parameter_exist(values.z):
            fixture["z"] = float(values.z)
        if fixture:
            res.append(fixture)
        return res, self.order_fixture

    def obtain_init_values(self, table, mission, readings, shop, section, fixture):
        '''
        Given a table, converts the apropiate message into a list of dictionaries
        :param table: table to target
        :param mission: Mission.msg
        :param readings: list of Readings.msg
        :param shop: Shop.msg
        :param section: Section.msg
        :param fixture: Fixture.msg
        :return: the list of dictionaries plus the order of the fixtures in the database
        '''
        if table == "Missions":
            res, order = self.mission_to_dict_list(mission)
        elif table == "Readings":
            res, order = self.readings_to_dict_list(readings)
        elif table == "Shops":
            res, order = self.shop_to_dict_list(shop)
        elif table == "Sections":
            res, order = self.section_to_dict_list(section)
        elif table == "Fixtures":
            res, order = self.fixture_to_dict_list(fixture)
        else:
            message = "[database_controller]: Table name not recognized "
            rospy.logerr(message)
            e = InvalidParameterDatabase()
            e.message = message
            raise e
        return res, order
