import unittest
import threading
from mock import patch, MagicMock
from rfid_controller.scripts.lib.sources.readings import Reading, DatabaseReading, ReadingQueue, ManageReadings


def fill_reading(reading):
    reading.antenna_port = 3
    reading.ts = 1111
    reading.rf_phase = 4
    reading.rssi = -74
    reading.freq = 44
    reading.mux1 = 0
    reading.mux2 = 2
    reading.epc = "000x2132132131"
    reading.device_id = "reader1"
    reading.robot_pose = [[0, 0, 0], [0, 0, 0, 0]]


class TestReading(unittest.TestCase):
    def setUp(self):
        self.reading = Reading()

    def test_eq_equal(self):
        # Prepare
        fill_reading(self.reading)
        reading2 = Reading()
        fill_reading(reading2)
        # Execute
        res = self.reading == reading2
        # Assert
        self.assertTrue(res)

    def test_eq_diff(self):
        # Prepare
        fill_reading(self.reading)
        reading2 = Reading()
        fill_reading(reading2)
        reading2.ts = 0010101011
        # Execute
        res = self.reading == reading2
        # Assert
        self.assertFalse(res)


class TestDatabaseReading(unittest.TestCase):
    def setUp(self):
        self.db_reading = DatabaseReading()

    def test_convert_reading_database_one(self):
        # Prepare
        reading = Reading()
        fill_reading(reading)
        # Execute
        res = self.db_reading.convert_readings_database([reading], "111-111-1-C1", "C1")
        print(res)
        # Assert
        self.assertEquals(res[0].id_reading, reading.epc + "-" + str(reading.ts))

    def test_convert_reading_database_more_one(self):
        # Prepare
        reading1 = Reading()
        fill_reading(reading1)
        reading2 = Reading()
        fill_reading(reading2)
        reading2.epc = "0xxx89aFss"
        # Execute
        res = self.db_reading.convert_readings_database([reading1, reading2], "111-111-1-C1", "C1")
        # Assert
        self.assertEquals(res[1].id_reading, reading2.epc + "-" + str(reading2.ts))

    def test_convert_reading_robot_pose_types(self):
        # Prepare
        reading = Reading()
        fill_reading(reading)
        # Execute
        res = self.db_reading.convert_readings_database([reading], "111-111-1-C1", "C1")
        # Assert
        self.assertEquals([type(res[0].id_reading), type(res[0].id_mission), type(res[0].id_section),
                           type(res[0].antenna_port), type(res[0].timestamp), type(res[0].rf_phase),
                           type(res[0].rssi), type(res[0].freq), type(res[0].mux1), type(res[0].mux2),
                           type(res[0].epc), type(res[0].device_id), type(res[0].robot_pose)],
                          [str, str, str, int, int, int, int, int, int, int, str, str, str])

    def test_convert_reading_robot_no_rf_phase(self):
        # Prepare
        reading = Reading()
        fill_reading(reading)
        reading.rf_phase = None
        # Execute
        res = self.db_reading.convert_readings_database([reading], "111-111-1-C1", "C1")
        # Assert
        self.assertEquals(res[0].rf_phase, 0)


class TestReadingQueue(unittest.TestCase):
    def setUp(self):
        self.queue = ReadingQueue()

    def test_push_element(self):
        # Prepare
        reading = Reading()
        # Execute
        self.queue.push(reading)
        # Assert
        self.assertEquals(reading, self.queue.queue[0])

    @patch('patterns.Observable.notify_observers')
    def test_push_notify(self, mock):
        # Prepare
        reading = Reading()
        # Execute
        self.queue.push(reading)
        # Assert
        mock.called_once()

    def test_pop(self):
        # Prepare
        reading = Reading()
        self.queue.push(reading)
        # Execute
        res = self.queue.pop()
        # Assert
        self.assertEquals(reading, res)

    def test_has_items_negative(self):
        # Prepare
        # Execute
        res = self.queue.has_items()
        # Assert
        self.assertFalse(res)

    def test_has_items_positive(self):
        # Prepare
        reading = Reading()
        self.queue.push(reading)
        # Execute
        res = self.queue.has_items()
        # Assert
        self.assertTrue(res)

    def test_pop_all_one(self):
        # Prepare
        reading = Reading()
        self.queue.push(reading)
        # Execute
        res = self.queue.pop_all()
        # Assert
        self.assertEqual(res, [reading])

    def test_pop_all_several(self):
        # Prepare
        reading1 = Reading()
        self.queue.push(reading1)
        reading2 = Reading()
        self.queue.push(reading2)
        reading3 = Reading()
        self.queue.push(reading3)
        # Execute
        res = self.queue.pop_all()
        # Assert
        self.assertEqual(res, [reading1, reading2, reading3])

    def test_pop_all_empty_origin(self):
        # Prepare
        reading1 = Reading()
        self.queue.push(reading1)
        reading2 = Reading()
        self.queue.push(reading2)
        reading3 = Reading()
        self.queue.push(reading3)
        # Execute
        res = self.queue.pop_all()
        # Assert
        self.assertEqual(self.queue.queue, list())


class TestManageReadings(unittest.TestCase):
    @patch('threading.Thread')
    @patch('rfid_controller.scripts.lib.sources.robot.Robot')
    @patch('rfid_controller.scripts.lib.sources.helpers.StoreReadings')
    def setUp(self, mock1, mock2, mock3):
        self.reading_queue = MagicMock()
        self.manage_readings = ManageReadings(self.reading_queue, "111-111-1-C1", "C1")
        self.manage_readings.db_reading = MagicMock()
        self.count_readings = MagicMock()
        self.robot = MagicMock()
        self.store_readings = MagicMock()

    def test_count_twist_store_call_count(self):
        # Prepare
        reading = Reading()
        reading.epc = "00x001AFF300020"
        self.manage_readings.reading_queue.pop_all = MagicMock(return_value=[reading])
        self.manage_readings.count_readings.count_unique_epc = MagicMock()
        # Execute
        self.manage_readings.count_twist_store()
        # Assert
        self.manage_readings.count_readings.count_unique_epc.assert_called_once()

    def test_count_twist_store_call_decide_twist(self):
        # Prepare
        reading = Reading()
        reading.epc = "00x001AFF300020"
        self.manage_readings.reading_queue.pop_all = MagicMock(return_value=[reading])
        self.manage_readings.robot.control_navigation_mode = MagicMock()
        # Execute
        self.manage_readings.count_twist_store()
        # Assert
        self.manage_readings.robot.control_navigation_mode.assert_called_once()

    def test_count_twist_store_call_convert_readings(self):
        # Prepare
        reading = Reading()
        reading.epc = "00x001AFF300020"
        self.manage_readings.reading_queue.pop_all = MagicMock(return_value=[reading])
        # Execute
        self.manage_readings.count_twist_store()
        # Assert
        self.manage_readings.db_reading.convert_readings_database.assert_called_once()

    def test_count_twist_store_call_store_readings(self):
        # Prepare
        reading = Reading()
        reading.epc = "00x001AFF300020"
        self.manage_readings.reading_queue.pop_all = MagicMock(return_value=[reading])
        # Execute
        self.manage_readings.count_twist_store()
        # Assert
        self.manage_readings.store_readings.store_readings.assert_called_once()

    def test_run_stoped(self):
        # Prepare
        stop_thread = True
        th = threading.Thread(target=self.manage_readings.run, args=(lambda: stop_thread,))
        # Execute
        th.start()
        th.join()
        # Assert
        self.assertFalse(th.is_alive())

    @patch('rfid_controller.scripts.lib.sources.readings.ManageReadings.count_twist_store')
    def test_run_stop(self, mock1):
        # Prepare
        stop_thread = False
        th = threading.Thread(target=self.manage_readings.run, args=(lambda: stop_thread,))
        # Execute
        th.start()
        stop_thread = True
        th.join()
        # Assert
        mock1.assert_called()

    @patch('rfid_controller.scripts.lib.sources.readings.ManageReadings.count_twist_store')
    def test_close_stop(self, mock1):
        # Prepare
        stop_thread = False
        self.manage_readings.th = threading.Thread(target=self.manage_readings.run,
                                                   args=(lambda: self.manage_readings.stop_thread,))
        self.manage_readings.th.start()
        # Execute
        self.manage_readings.close()
        # Assert
        self.assertFalse(self.manage_readings.th.is_alive())
