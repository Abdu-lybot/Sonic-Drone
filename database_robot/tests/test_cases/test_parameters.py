#!/usr/bin/env python
import unittest
from mock import patch
from self_exceptions import InvalidParameterDatabase
from database_robot.scripts.lib.sources.parameters import ParameterHelper


class MissionMock:
    def __init__(self):
        self.id_mission = "1111-111-1-C1"
        self.start = "13213213-123213"
        self.finish = "12333-5322"
        self.sections = "[C1, C2]"
        self.task = "inventory"
        self.processed = "False"


class MissionMockEmpty:
    def __init__(self):
        self.id_mission = "None"
        self.start = ""
        self.finish = ""
        self.sections = ""
        self.task = ""
        self.processed = ""


class ShopMock:
    def __init__(self):
        self.id_shop = "1"
        self.name = "test_shop"
        self.description = "Some description"
        self.sections = "[C1, C2]"


class ShopMockEmpty:
    def __init__(self):
        self.id_shop = "None"
        self.name = ""
        self.description = ""
        self.sections = ""


class SectionMock:
    def __init__(self):
        self.id_section = "C1"
        self.name = "test_section"
        self.qr_docking = "QR_DOCKING"
        self.qr_section = "QR_SECTION"
        self.map_path = "/map/path"
        self.description = "Some description"
        self.id_shop = "1"
        self.waypoint_path = "/waypoint/path"


class SectionMockEmpty:
    def __init__(self):
        self.id_section = "None"
        self.name = ""
        self.qr_docking = ""
        self.qr_section = ""
        self.map_path = ""
        self.description = ""
        self.id_shop = ""
        self.waypoint_path = ""


class ReadingMock:
    def __init__(self):
        self.id_reading = "eee000x99-11111-11"
        self.id_mission = "11111-11"
        self.id_section = "C1"
        self.antenna_port = "1"
        self.timestamp = "11"
        self.rf_phase = "45"
        self.rssi = "-74"
        self.freq = "421321"
        self.mux1 = "1"
        self.mux2 = "0"
        self.epc = "eee000x99"
        self.device_id = "reader-01"
        self.robot_pose = "[[0, 0, 0], [1, 1, 1]]"


class ReadingMockEmpty:
    def __init__(self):
        self.id_reading = "None"
        self.id_mission = ""
        self.id_section = ""
        self.antenna_port = ""
        self.timestamp = ""
        self.rf_phase = ""
        self.rssi = ""
        self.freq = ""
        self.mux1 = ""
        self.mux2 = ""
        self.epc = ""
        self.device_id = ""
        self.robot_pose = ""


class FixtureMock:
    def __init__(self):
        self.id_fixture = "1-C1-Colette1"
        self.id_shop = "1"
        self.id_section = "C1"
        self.name = "Colette1"
        self.area = "W1"
        self.fix_group = "Otros 1"
        self.ref_tag = "000xff23213"
        self.x = "-0.33"
        self.y = "4.2"
        self.z = None

class FixtureMockEmpty:
    def __init__(self):
        self.id_fixture = "None"
        self.id_shop = ""
        self.id_section = ""
        self.name = ""
        self.area = ""
        self.fix_group = ""
        self.ref_tag = ""
        self.x = ""
        self.y = ""
        self.z = ""


class TestParameterTransform(unittest.TestCase):
    @patch('rospy.get_param')
    def setUp(self, mock):
        self.param = ParameterHelper()

    def test_mission_to_dict_list_all(self):
        #Prepare
        mission = MissionMock()
        #Execute
        res, order = self.param.mission_to_dict_list(mission)
        #Assert
        self.assertEquals(res, [{"id_mission": mission.id_mission, "start": mission.start, "finish": mission.finish,
                                "sections": mission.sections, "task": mission.task, "processed": False}])

    def test_mission_to_dict_list_all_emtpy(self):
        #Prepare
        mission = MissionMockEmpty()
        #Execute
        res, order = self.param.mission_to_dict_list(mission)
        #Assert
        self.assertEquals(res, [])

    def test_shop_to_dict_list_all(self):
        #Prepare
        shop = ShopMock()
        #Execute
        res, order = self.param.shop_to_dict_list(shop)
        #Assert
        self.assertEquals(res, [{"id_shop": 1, "name": shop.name, "description": shop.description,
                                "sections": shop.sections}])

    def test_shop_to_dict_list_all_emtpy(self):
        #Prepare
        shop = ShopMockEmpty()
        #Execute
        res, order = self.param.shop_to_dict_list(shop)
        #Assert
        self.assertEquals(res, [])

    def test_section_to_dict_list_all(self):
        #Prepare
        section = SectionMock()
        #Execute
        res, order = self.param.section_to_dict_list(section)
        #Assert
        self.assertEquals(res, [{"id_section": section.id_section, "name": section.name,
                                "qr_docking": section.qr_docking, "qr_section": section.qr_section,
                                "map_path": section.map_path, "description": section.description,
                                "id_shop": 1, "waypoint_path": section.waypoint_path}])

    def test_section_to_dict_list_all_emtpy(self):
        #Prepare
        section = SectionMockEmpty()
        #Execute
        res, order = self.param.section_to_dict_list(section)
        #Assert
        self.assertEquals(res, [])

    def test_readings_to_dict_list_all(self):
        #Prepare
        reading = ReadingMock()
        #Execute
        res, order = self.param.readings_to_dict_list([reading])
        #Assert
        self.assertEquals(res, [{"id_reading": reading.id_reading, "id_mission": reading.id_mission,
                                 "id_section": reading.id_section, "antenna_port": 1, "timestamp": 11, "rf_phase": 45,
                                 "rssi": -74, "freq": 421321, "mux1": 1, "mux2": 0, "epc": reading.epc,
                                 "device_id": reading.device_id, "robot_pose": reading.robot_pose}])

    def test_readings_to_dict_list_all_emtpy(self):
        #Prepare
        readings = [ReadingMockEmpty()]
        #Execute
        res, order = self.param.readings_to_dict_list(readings)
        #Assert
        self.assertEquals(res, [])

    def test_fixture_to_dict_list_all(self):
        #Prepare
        fixture = FixtureMock()
        #Execute
        res, order = self.param.fixture_to_dict_list(fixture)
        #Assert
        self.assertEquals(res, [{"id_fixture": fixture.id_fixture, "id_shop": 1,
                                 "id_section": fixture.id_section, "name": fixture.name, "area": fixture.area,
                                 "fix_group": fixture.fix_group, "ref_tag": fixture.ref_tag, "x": -0.33, "y": 4.2}])

    def test_fixture_to_dict_list_all_emtpy(self):
        #Prepare
        fixture = FixtureMockEmpty()
        #Execute
        res, order = self.param.fixture_to_dict_list(fixture)
        #Assert
        self.assertEquals(res, [])

    @patch('database_robot.scripts.lib.sources.parameters.ParameterHelper.mission_to_dict_list',
           return_value=[None, None])
    def test_obtain_values_mission(self, mock):
        #Prepare
        table = "Missions"
        mission = MissionMock()
        #Execute
        res, order = self.param.obtain_init_values(table, mission, None, None, None, None)
        #Assert
        mock.assert_called_once_with(mission)

    @patch('database_robot.scripts.lib.sources.parameters.ParameterHelper.readings_to_dict_list',
           return_value=[None, None])
    def test_obtain_values_readings(self, mock):
        #Prepare
        table = "Readings"
        readings = [ReadingMock()]
        #Execute
        res, order = self.param.obtain_init_values(table, None, readings, None, None, None)
        #Assert
        mock.assert_called_once_with(readings)

    @patch('database_robot.scripts.lib.sources.parameters.ParameterHelper.shop_to_dict_list',
           return_value=[None, None])
    def test_obtain_values_shops(self, mock):
        #Prepare
        table = "Shops"
        shop = ShopMock()
        #Execute
        res, order = self.param.obtain_init_values(table, None, None, shop, None, None)
        #Assert
        mock.assert_called_once_with(shop)

    @patch('database_robot.scripts.lib.sources.parameters.ParameterHelper.section_to_dict_list',
           return_value=[None, None])
    def test_obtain_values_section(self, mock):
        #Prepare
        table = "Sections"
        section = SectionMock()
        #Execute
        res, order = self.param.obtain_init_values(table, None, None, None, section, None)
        #Assert
        mock.assert_called_once_with(section)

    @patch('database_robot.scripts.lib.sources.parameters.ParameterHelper.fixture_to_dict_list',
           return_value=[None, None])
    def test_obtain_values_fixture(self, mock):
        #Prepare
        table = "Fixtures"
        fixture = FixtureMock()
        #Execute
        res, order = self.param.obtain_init_values(table, None, None, None, None, fixture)
        #Assert
        mock.assert_called_once_with(fixture)

    def test_obtain_values_exception(self):
        #Prepare
        table = "Bad_table_name"
        #Execute
        try:
            res, order = self.param.obtain_init_values(table, None, None, None, None, None)
        except InvalidParameterDatabase:
            pass
        #Assert
        self.assertRaises(Exception)

    def test_obtain_values_exception_message(self):
        #Prepare
        table = "Bad_table_name"
        #Execute
        try:
            res, order = self.param.obtain_init_values(table, None, None, None, None, None)
        except InvalidParameterDatabase as e:
            pass
        #Assert
        self.assertTrue(e.message)
