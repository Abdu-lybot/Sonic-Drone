import rospy
import advannet_rest
import socket
import helpers
from threading import Thread
import robot
from readings import Reading


class Readers:
    '''
    Class that controls the readers
    '''
    def __init__(self, reading_queue):
        self.stop_readers = False
        self.readers = list()
        self.reading_queue = reading_queue
        for reader in rospy.get_param('~rfid/readers'):
            # Obtain readers URLS
            url = 'http://' + reader['reader_ip'] + ':' + str(reader['rest_port']) + '/devices/' + \
                  reader['reader_id']
            #Create sockets
            rospy.logdebug("[RFIDController]: setting up reader %s with url %s", str(reader['reader_id']), url)
            soc = Socket(reader['reader_ip'], reader['data_port'], self.reading_queue)
            self.readers.append({"url": url,
                                 "socket": soc,
                                 "th": None})

    def set_up(self, task):
        '''
        Set up the readers
        :param task: 'inventory' or 'location'
        :return:
        '''
        #Init variables
        session = ''
        power = ''
        #Configure readers
        if task == 'location':
            session = 'S0'
            power = '20'
        elif task == 'inventory':
            session = 'S1'
            power = '30'
        rospy.logdebug('[RFIDController]: Setting readers to session %s and power %sDB', session, power)
        for reader in self.readers:
            #Set session and power
            resp1 = advannet_rest.set_session(reader["url"], session)
            resp2 = advannet_rest.set_rf_read_power(reader["url"], power)
            if not resp1 or not resp2:
                rospy.logerr('[RFIDController]: Could not set power and session')
                return False
            # Connect socket
            reader["socket"].connect()
        return True

    def start(self):
        '''
        Start the readers
        :return:
        '''
        rospy.logdebug('[RFIDController]: Starting readers')
        for reader in self.readers:
            #Start reading from socket
            reader["th"] = Thread(target=reader["socket"].read, args=(lambda: self.stop_readers,))
            reader["th"].start()
            #Start reader
            resp = advannet_rest.start_reader(reader["url"])
            if not resp:
                rospy.logerr('[RFIDController]: Could not start the reader')
                return False
        while not self.stop_readers:
            rospy.sleep(0.2)
        return True

    def stop(self):
        '''
        Stop the readers
        :return:
        '''
        rospy.logdebug('[RFIDController]: Closing readers')
        for reader in self.readers:
            #Stop reading
            self.stop_readers = True
            if reader["th"]:
                reader["th"].join()
                #Stop readers
                resp = advannet_rest.stop_reader(reader["url"])
            #Close socket
            reader["socket"].close()
            if not resp:
                rospy.logerr('[RFIDController]: Could not stop the reader')
                return False
        return True


class Socket:
    '''
    Class to control the socket of the readers and normalize its output
    '''
    def __init__(self, host, port, reading_queue):
        self.host = host
        self.port = port
        self.socket = None
        self.reading_queue = reading_queue

    @staticmethod
    def _inventory_message_parser(stream):
        '''
        Parses a reading from a stream, discard other messages
        :param stream:
        :return:
        '''
        xml_stream = ""
        line = stream.readline()
        #Detect start of inventory
        if "<inventory>" in line:
            #Detect blank line
            while len(line.strip()) != 0:
                xml_stream = xml_stream + line
                line = stream.readline()
            xml_stream + line
            return xml_stream
        return False

    def connect(self):
        '''
        Connect to the socket
        :return:
        '''
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.host, self.port))

    def read(self, stop):
        '''
        To be launched as a thread. Reads from the socket, parses a reading and stores it
        :param stop:
        :return:
        '''
        xml_parser = helpers.XMLParser()
        robot_sing = robot.Robot()
        #Convert stream into a file so we can use readline
        file_stream = self.socket.makefile('r')
        while not stop():
            resp = self._inventory_message_parser(file_stream)
            if resp:
                reading = Reading()
                #Parse xml
                xml_parser.parse_inventory(resp, reading)
                #Obtain robot position
                reading.robot_pose = robot_sing.get_pose()
                reading.print_reading()
                #Push in the reading queue
                self.reading_queue.push(reading)
                
    def close(self):
        '''
        Close the socket
        :return:
        '''
        self.socket.close()
