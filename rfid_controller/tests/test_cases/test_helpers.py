import unittest
import os
from mock import patch, MagicMock
from rfid_controller.scripts.lib.sources.helpers import XMLParser, StoreReadings, CountReadings
from rfid_controller.scripts.lib.sources.readings import Reading
from database_robot.msg import Reading as MessageReading


def get_reading():
    reading = Reading()
    reading.antenna_port = 1
    reading.ts = 1603353691233
    reading.rf_phase = 42
    reading.rssi = -74
    reading.freq = 866829
    reading.mux1 = 0
    reading.mux2 = 0
    reading.epc = "303499f81415f1b496df661a"
    reading.device_id = "reader-01"
    return reading


class TestXMLParser(unittest.TestCase):
    def setUp(self):
        self.xml_parser = XMLParser()

    def test_parse_inventory(self):
        #Prepare
        reading_check = get_reading()
        dirname = os.path.dirname(__file__)
        f = open(dirname + "/../test_data/inventory_only.txt", 'r')
        string = f.read()
        reading = Reading()
        #Execute
        self.xml_parser.parse_inventory(string, reading)
        #Assert
        self.assertEquals(reading, reading_check)

    def test_parse_inventory_several(self):
        # Prepare
        reading_check = get_reading()
        dirname = os.path.dirname(__file__)
        f = open(dirname + "/../test_data/several_readings.txt", 'r')
        string = f.read()
        reading = Reading()
        # Execute
        self.xml_parser.parse_inventory(string, reading)
        # Assert
        self.assertEquals([reading.rf_phase, reading.antenna_port, reading.mux1, reading.mux2, reading.freq],
                          [109, 3, 0, 0, 865549])


class TestStoreReadings(unittest.TestCase):
    @patch('rospy.ServiceProxy')
    def setUp(self, mock1):
        self.store = StoreReadings()

    def test_store_readings_true(self):
        #Prepare
        self.store.database_controller = MagicMock()
        ms_reading = MessageReading()
        #Execute
        self.store.store_readings([ms_reading])
        #Assert
        self.store.database_controller.assert_called_once_with("INSERT", "Readings", None, [ms_reading], None, None,
                                                               None)


class TestCountReadings(unittest.TestCase):
    def setUp(self):
        self.count = CountReadings()

    def test_count_unique_epc(self):
        #Prepare
        read1 = Reading()
        read1.epc = "1111"
        read2 = Reading()
        read2.epc = "0000"
        read3 = Reading()
        read3.epc = "1111"
        #Execute
        num = self.count.count_unique_epc([read1, read2, read3])
        #Assert
        self.assertEquals(num, 2)
