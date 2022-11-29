#!/usr/bin/env python
import unittest
import os
import sqlite3
from self_exceptions import InvalidQueryDatabase
from database_robot.scripts.lib.sources.database_operations import DatabaseOperations
from mock import patch


MOCK_DB_PATH = os.path.dirname(__file__) + "/../test_data/robot_test.db"


def delete_shop(db):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    c.execute('PRAGMA foreign_keys = ON')
    c.execute('DELETE from Shops')
    conn.commit()


class TestDatabaseOperations(unittest.TestCase):
    @patch('rospy.get_param', return_value=MOCK_DB_PATH)
    def setUp(self, mock_param):
       self.db_op = DatabaseOperations()

    def tearDown(self):
        delete_shop(MOCK_DB_PATH)

    def test_execute_query_insert_one(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = [(1, "test_shop", "some description", "[C1,C2]")]
        #Execute
        self.db_op._execute_query(query, values)
        #Assert
        conn = sqlite3.connect(MOCK_DB_PATH)
        c = conn.cursor()
        c.execute('SELECT * FROM Shops')
        line = c.fetchone()
        self.assertEquals(line, values[0])

    def test_execute_query_insert_not_all_values(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = [(1, "test_shop", "some description", None)]
        #Execute
        self.db_op._execute_query(query, values)
        #Assert
        conn = sqlite3.connect(MOCK_DB_PATH)
        c = conn.cursor()
        c.execute('SELECT * FROM Shops')
        line = c.fetchone()
        self.assertEquals(line, values[0])

    def test_execute_query_insert_many(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = [(1, "test_shop", "some description", "[C1,C2]"), (2, "test_shop2", "some description2", "[C3,C4]")]
        #Execute
        self.db_op._execute_query(query, values)
        #Assert
        conn = sqlite3.connect(MOCK_DB_PATH)
        c = conn.cursor()
        c.execute('SELECT * FROM Shops')
        lines = c.fetchall()
        self.assertEquals(lines, values)

    def test_execute_query_insert_repeated_shop(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = [(1, "test_shop", "some description", "[C1,C2]")]
        self.db_op._execute_query(query, values)
        #Execute
        try:
            self.db_op._execute_query(query, values)
        except InvalidQueryDatabase:
            pass
        #Assert
        self.assertRaises(Exception)

    def test_execute_query_delete_specific(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = (1, "test_shop", "some description", "[C1,C2]")
        self.db_op._execute_query(query, values)
        query = 'DELETE FROM Shops WHERE id_shop=?'
        values = (1, )
        #Execute
        self.db_op._execute_query(query, values)
        #Assert
        conn = sqlite3.connect(MOCK_DB_PATH)
        c = conn.cursor()
        c.execute('SELECT * FROM Shops')
        line = c.fetchone()
        self.assertEquals(line, None)

    def test_execute_query_delete_all(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = (1, "test_shop", "some description", "[C1,C2]")
        self.db_op._execute_query(query, values)
        query = 'DELETE FROM Shops'
        value = tuple()
        #Execute
        self.db_op._execute_query(query, value)
        #Assert
        conn = sqlite3.connect(MOCK_DB_PATH)
        c = conn.cursor()
        c.execute('SELECT * FROM Shops')
        line = c.fetchone()
        self.assertEquals(line, None)

    def test_execute_query_select(self):
        #Prepare
        query = "INSERT INTO Shops VALUES (?,?,?,?)"
        values = (1, "test_shop", "some description", "[C1,C2]")
        self.db_op._execute_query(query, values)
        query = 'SELECT * FROM Shops'
        value = tuple()
        #Execute
        data = self.db_op._execute_query(query, value)
        #Assert
        self.assertEquals(data, [values])


    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_insert_many(self, mock_execute):
        # Prepare
        table = "Shops"
        values = [
            {"id_shop": 1,
            "name": "test_shop",
            "description": "some description",
            "sections": "[C1, C2]"},
            {"id_shop": 2,
             "name": "test_shop2",
             "description": "some description2",
             "sections": "[C3, C4]"}]
        order = ["id_shop", "name", "description", "sections"]
        check_values = [(1, "test_shop", "some description", "[C1, C2]"),
                        (2, "test_shop2", "some description2", "[C3, C4]")]
        # Execute
        self.db_op.insert(table, values, order)
        # Assert
        mock_execute.assert_called_once_with("INSERT INTO Shops VALUES (?,?,?,?)", check_values)

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_insert_one(self, mock_execute):
        # Prepare
        table = "Shops"
        values = [
            {"id_shop": 1,
             "name": "test_shop",
             "description": "some description",
             "sections": "[C1, C2]"}]
        check_values = [(1, "test_shop", "some description", "[C1, C2]")]
        order = ["id_shop", "name", "description", "sections"]
        # Execute
        self.db_op.insert(table, values, order)
        # Assert
        mock_execute.assert_called_once_with("INSERT INTO Shops VALUES (?,?,?,?)", check_values)

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_insert_no_fk(self, mock_execute):
        # Prepare
        table = "Readings"
        values = [{
            "id_reading": 111111-111, "id_mission": "111-111-1-C1", "id_section": "C1", "antenna_port": 1,
            "timestamp": 1111111, "rf_phase": 45, "rssi": -74, "freq": 866829, "mux1": 0, "mux2": 1,
            "epc": "2782313XAA2", "device_id": "reader-01", "robot_pose": "[[0,0,0],[0,0,0]]"}]
        order = ["id_reading", "id_mission", "id_section", "antenna_port", "timestamp", "rf_phase", "rssi", "freq",
                 "mux1", "mux2", "epc", "device_id", "robot_pose"]
        # Execute
        try:
            res = self.db_op.insert(table, values, order)
        except InvalidQueryDatabase:
            pass
        # Assert
        self.assertRaises(Exception)

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_insert_shop_no_section(self, mock_execute):
        # Prepare
        table = "Shops"
        values = [
            {"id_shop": 1,
             "name": "test_shop",
             "description": "some description"
             }]
        check_values = [(1, "test_shop", "some description", None)]
        order = ["id_shop", "name", "description", "sections"]
        # Execute
        self.db_op.insert(table, values, order)
        # Assert
        mock_execute.assert_called_once_with("INSERT INTO Shops VALUES (?,?,?,?)", check_values)

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_delete_where(self, mock_execute):
        #Prepare
        table = "Shops"
        values = [{
            "id_shop": 1
        }]
        #Execute
        self.db_op.delete(table, values)
        #Assert
        mock_execute.assert_called_once_with("DELETE FROM Shops WHERE id_shop=?", (1, ))

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_delete_no_where(self, mock_execute):
        #Prepare
        table = "Shops"
        #Execute
        self.db_op.delete(table, None)
        #Assert
        mock_execute.assert_called_once_with("DELETE FROM Shops", tuple())

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_delete_where_several(self, mock_execute):
        #Prepare
        table = "Shops"
        values = [{
            "name": "some_name",
            "sections": "[C1,C2]"
        }]
        #Execute
        self.db_op.delete(table, values)
        #Assert
        mock_execute.assert_called_once_with("DELETE FROM Shops WHERE sections=? AND name=?",
                                             ("[C1,C2]", "some_name"))

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_select_where(self, mock_execute):
        #Prepare
        table = "Shops"
        values = [{
            "id_shop": 1
        }]
        #Execute
        self.db_op.select(table, values, [])
        mock_execute.assert_called_once_with("SELECT * FROM Shops WHERE id_shop=?", (1, ))

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_select_no_where(self, mock_execute):
        #Prepare
        table = "Shops"
        #Execute
        self.db_op.select(table, None, [])
        #Assert
        mock_execute.assert_called_once_with("SELECT * FROM Shops", tuple())


    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_select_where_several(self, mock_execute):
        #Prepare
        table = "Shops"
        values = [{
            "name": "some_name",
            "sections": "[C1,C2]"
        }]
        #Execute
        self.db_op.select(table, values, [])
        #Assert
        mock_execute.assert_called_once_with("SELECT * FROM Shops WHERE sections=? AND name=?",
                                             ("[C1,C2]", "some_name"))

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_select_some_fields(self, mock_execute):
        #Prepare
        table = "Shops"
        #Execute
        self.db_op.select(table, None, ["name"])
        #Assert
        mock_execute.assert_called_once_with("SELECT name FROM Shops", tuple())

    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_select_distinct(self, mock_execute):
        #Prepare
        table = "Shops"
        values = ["name"]
        #Execute
        self.db_op.select_distinct(table, None, values)
        #Assert
        mock_execute.assert_called_once_with("SELECT DISTINCT name FROM Shops", tuple())


    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_update_simple(self, mock_execute):
        #Prepare
        table = "Shops"
        values = [{
            "name": "some_name",
            "sections": "[C1,C2]"
        }]
        update_values = ["name"]
        #Execute
        self.db_op.update(table, values, update_values)
        #Assert
        mock_execute.assert_called_once_with("UPDATE Shops SET name=? WHERE sections=?",

                                             ("some_name", "[C1,C2]"))
    @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    def test_update_complex(self, mock_execute):
        #Prepare
        table = "Missions"
        values = [{
            "finish": "2222-22",
            "processed": True,
            "id_mission": 1,
            "start": "1111-11"
        }]
        update_values = ["finish", "processed"]
        #Execute
        self.db_op.update(table, values, update_values)
        #Assert
        mock_execute.assert_called_once_with("UPDATE Missions SET finish=?, processed=? WHERE start=? AND id_mission=?",
                                             ("2222-22", True, "1111-11", 1))

    # @patch('database_robot.scripts.lib.sources.database_operations.DatabaseOperations._execute_query')
    # def test_select_distinct_where_one(self, mock_execute):
    #     #Prepare
    #     table = "Shops"
    #     values = [{
    #         "id_shop": 1
    #     }]
    #     select_fields = ["name"]
    #     #Execute
    #     self.db_op.select(table, values)
    #     mock_execute.assert_called_once_with("SELECT * FROM Shops WHERE id_shop=?", (1, ))
