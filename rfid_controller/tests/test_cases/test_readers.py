import unittest
import os
import socket
from threading import Thread
from mock import patch, MagicMock, create_autospec
from rfid_controller.scripts.lib.sources.readers import Readers, Socket
from rfid_controller.scripts.lib.sources.helpers import XMLParser


class ServerMock:
    def __init__(self, host, port):
        self.conn = None
        self.addr = None
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(1)
        self.server.settimeout(5)

    def accept_connection(self):
        self.conn, self.addr = self.server.accept()

    def close(self):
        self.server.close()


class TestSocket(unittest.TestCase):
    def setUp(self):
        host = 'localhost'
        port = 65448
        self.socket = Socket(host, port, MagicMock())
        #create mock server
        self.mock = ServerMock(host, port)

    def tearDown(self):
        self.mock.close()

    def test_connect(self):
        #Prepare
        th = Thread(target=self.mock.accept_connection)
        th.start()
        #Execute
        self.socket.connect()
        th.join()
        #Assert
        self.assertEquals(self.mock.addr[0], '127.0.0.1')

    def test__inventory_message_parser_true(self):
        #Prepare
        dirname = os.path.dirname(__file__)
        f1 = open(dirname + "/../test_data/inventory_only_check.txt", 'r')
        check = f1.read()
        f1.close()
        f2 = open(dirname + "/../test_data/inventory_only.txt", 'r')
        #Execute
        res = self.socket._inventory_message_parser(f2)
        #Assert
        self.assertEquals(res, check)

    def test__inventory_message_parser_false(self):
        #Prepare
        dirname = os.path.dirname(__file__)
        f = open(dirname + "/../test_data/no_inventory.txt", 'r')
        #Execute
        res = self.socket._inventory_message_parser(f)
        #Assert
        self.assertFalse(res)

    @patch('rfid_controller.scripts.lib.sources.readers.Socket._inventory_message_parser', return_value=False)
    @patch('rfid_controller.scripts.lib.sources.helpers.XMLParser', return_value=create_autospec(XMLParser))
    @patch('rfid_controller.scripts.lib.sources.robot.Robot', return_value=MagicMock())
    def test_read_no_resp(self, mock1, mock2, mock3):
        #Prepare
        self.socket.socket = MagicMock()
        stop = False
        #Execute
        th = Thread(target=self.socket.read, args=(lambda: stop, ))
        th.start()
        stop = True
        th.join()
        #Assert
        mock2.parse_inventory.assert_not_called()


class TestReaders(unittest.TestCase):
    @patch('rospy.get_param')
    def setUp(self, mock1):
        self.readers = Readers(MagicMock())
        soc = MagicMock()
        self.readers.readers = [{"url": 'http://reader-01',
                                "socket": soc}]

    @patch('advannet_rest.set_session', return_value=True)
    @patch('advannet_rest.set_rf_read_power', return_value=True)
    def test_set_up_location_power_call(self, mock1, mock2):
        #Prepare
        #Execute
        self.readers.set_up('location')
        #Assert
        mock1.assert_called_once_with('http://reader-01', '20')

    @patch('advannet_rest.set_session', return_value=True)
    @patch('advannet_rest.set_rf_read_power', return_value=True)
    def test_set_up_location_session_call(self, mock1, mock2):
        #Prepare
        #Execute
        self.readers.set_up('location')
        #Assert
        mock2.assert_called_once_with('http://reader-01', 'S0')

    @patch('advannet_rest.set_session', return_value=True)
    @patch('advannet_rest.set_rf_read_power', return_value=True)
    def test_set_up_inventory_power_call(self, mock1, mock2):
        #Prepare
        #Execute
        self.readers.set_up('inventory')
        #Assert
        mock1.assert_called_once_with('http://reader-01', '30')

    @patch('advannet_rest.set_session', return_value=True)
    @patch('advannet_rest.set_rf_read_power', return_value=True)
    def test_set_up_inventory_session_call(self, mock1, mock2):
        #Prepare
        #Execute
        self.readers.set_up('inventory')
        #Assert
        mock2.assert_called_once_with('http://reader-01', 'S2')

    @patch('advannet_rest.set_session', return_value=True)
    @patch('advannet_rest.set_rf_read_power', return_value=True)
    def test_set_up_session_true(self, mock1, mock2):
        #Prepare
        #Execute
        resp = self.readers.set_up('inventory')
        #Assert
        self.assertTrue(resp)

    @patch('advannet_rest.set_session', return_value=False)
    @patch('advannet_rest.set_rf_read_power', return_value=True)
    def test_set_up_session_false(self, mock1, mock2):
        #Prepare
        #Execute
        resp = self.readers.set_up('inventory')
        #Assert
        self.assertFalse(resp)

    @patch('advannet_rest.set_session', return_value=True)
    @patch('advannet_rest.set_rf_read_power', return_value=False)
    def test_set_up_power_false(self, mock1, mock2):
        #Prepare
        #Execute
        resp = self.readers.set_up('inventory')
        #Assert
        self.assertFalse(resp)

    @patch('advannet_rest.start_reader', return_value=True)
    def test_start_call(self, mock1):
        #Prepare
        self.readers.stop_readers = True
        #Execute
        resp = self.readers.start()
        #Assert
        mock1.assert_called_once_with('http://reader-01')

    @patch('threading.Thread')
    @patch('advannet_rest.start_reader', return_value=True)
    def test_start_true(self, mock1, mock2):
        #Prepare
        self.readers.stop_readers = True
        #Execute
        resp = self.readers.start()
        #Assert
        self.assertTrue(resp)

    @patch('threading.Thread')
    @patch('advannet_rest.start_reader', return_value=False)
    def test_start_false(self, mock1, mock2):
        #Prepare
        self.readers.stop_readers = True
        #Execute
        resp = self.readers.start()
        #Assert
        self.assertFalse(resp)

    @patch('advannet_rest.stop_reader', return_value=True)
    def test_stop_call(self, mock1):
        #Prepare
        self.readers.readers[0]["th"] = MagicMock()
        #Execute
        resp = self.readers.stop()
        #Assert
        mock1.assert_called_once_with('http://reader-01')

    @patch('advannet_rest.stop_reader', return_value=True)
    def test_stop_true(self, mock1):
        #Prepare
        self.readers.readers[0]["th"] = MagicMock()
        #Execute
        resp = self.readers.stop()
        #Assert
        self.assertTrue(resp)

    @patch('advannet_rest.stop_reader', return_value=False)
    def test_stop_false(self, mock1):
        #Prepare
        self.readers.readers[0]["th"] = MagicMock()
        #Execute
        resp = self.readers.stop()
        #Assert
        self.assertFalse(resp)

    @patch('advannet_rest.stop_reader', return_value=False)
    def test_stop_socket_close(self, mock1):
        #Prepare
        self.readers.readers[0]["th"] = MagicMock()
        close_mock = MagicMock()
        self.readers.readers[0]["socket"].close = close_mock
        #Execute
        resp = self.readers.stop()
        #Assert
        close_mock.assert_called_once()
