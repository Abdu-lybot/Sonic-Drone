#!/usr/bin/env python

import unittest
import rosunit
from test_cases.test_database_operations import TestDatabaseOperations
from test_cases.test_parameters import TestParameterTransform


class TestSuiteDatabase(unittest.TestSuite):
    def __init__(self):
        super(TestSuiteDatabase, self).__init__()
        self.addTest(TestDatabaseOperations)
        self.addTest(TestParameterTransform)


if __name__ == '__main__':
    rosunit.unitrun('test_database_robot', 'test_database_operations',
                    'test.test_database_robot.TestSuiteDatabaseoperations')
