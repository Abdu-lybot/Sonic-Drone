#!/usr/bin/env python
import unittest
import rosunit
from test_cases.test_readers import TestReaders, TestSocket
from test_cases.test_helpers import TestXMLParser, TestStoreReadings, TestCountReadings
from test_cases.test_readings import TestReading, TestDatabaseReading, TestReadingQueue, TestManageReadings
from test_cases.test_robot import TestRobot


class TestSuiteHelpers(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteHelpers, self).__init__()
        self.addTest(TestXMLParser)
        self.addTest(TestStoreReadings)
        self.addTest(TestCountReadings)
        #Fixme: new suite
        self.addTest(TestRobot)


class TestSuiteReading(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteReading, self).__init__()
        self.addTest(TestReading)
        self.addTest(TestDatabaseReading)
        self.addTest(TestReadingQueue)
        self.addTest(TestManageReadings)


class TestSuiteReaders(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteReaders, self).__init__()
        self.addTest(TestReaders)
        self.addTest(TestSocket)


if __name__ == '__main__':
    rosunit.unitrun('test_rfid_controller', 'test_suite_helpers',
                    'test.test_rfid_controller.TestSuiteHelpers')
    rosunit.unitrun('test_rfid_controller', 'test_suite_reading',
                    'test.test_rfid_controller.TestSuiteReading')
    rosunit.unitrun('test_rfid_controller', 'test_suite_readers',
                    'test.test_rfid_controller.TestSuiteReaders')
