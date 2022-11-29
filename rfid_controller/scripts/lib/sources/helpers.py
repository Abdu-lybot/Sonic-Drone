import rospy
import xml.etree.ElementTree as ET
from database_robot.srv import DatabaseRobot
from database_robot.msg import ReadingList


class XMLParser:
    '''
    Class to parse XML recieved from the readers and obtain reading objects
    '''
    def __init__(self):
        pass

    @staticmethod
    def parse_inventory(string, reading):
        '''
        Obtains an object of type Reading from a string
        :param string: string with the xml
        :param reading: reading to fullfill
        :return: object of type Reading
        '''
        #Obtain xml elements
        root = ET.fromstring(string)
        inventory = root.find('data').find('inventory')
        item = inventory.find('items').find('item')
        props = item.find('data').find('props').findall('prop')
        props_text = [prop.text for prop in props]
        reading.ts = int(item.find('ts').text)
        reading.epc = item.find('epc').text
        reading.device_id = item.find('deviceId').text
        for prop in props_text:
            if prop.find('RSSI') != -1:
                reading.rssi = int(prop.split(':')[1].split(',')[0])
            elif prop.find('RF_PHASE') != -1:
                reading.rf_phase = int(prop.split(':')[1].split(',')[0])
            elif prop.find('ANTENNA_PORT') != -1:
                reading.antenna_port = int(prop.split(':')[1].split(',')[0])
            elif prop.find('MUX1') != -1:
                reading.mux1 = int(prop.split(':')[1].split(',')[0])
            elif prop.find('MUX2') != -1:
                reading.mux2 = int(prop.split(':')[1].split(',')[0])
            elif prop.find('FREQ') != -1:
                reading.freq = int(prop.split(':')[1].split(',')[0])
        return reading


class StoreReadings:
    '''
    Class that manages the database storing
    '''
    def __init__(self):
        #Start database_controller service
        self.database_controller = rospy.ServiceProxy(rospy.get_namespace() + '/database_robot/database_controller', DatabaseRobot)

    def store_readings(self, db_readings):
        '''
        Store a list of readings
        :param db_readings: list of readings
        :return:
        '''
        # Execute service
        res = self.database_controller("INSERT", "Readings", None, db_readings, None, None, None, None)
        if not res.status:
            rospy.logerr("[TaskManager]: Error inserting readings. Error message %s", res.message)


class CountReadings:
    def __init__(self):
        self.unique_readings = dict()

    def count_unique_epc(self, readings):
        '''
        Return the number of unique epcs from a list of readings
        :param readings: list of readings
        :return: number of not previously read epcs
        '''
        count = 0
        # Check if the epc has been already read
        for reading in readings:
            if reading.epc not in self.unique_readings:
                self.unique_readings[reading.epc] = True
                count = count + 1
        return count


class ReadingListPublisher:
    """
    Sleeps until it receives a call from the step manager. After that it publishes the list of readings.
    """
    def __init__(self):
        # Readings publisher
        self.readings_publisher = rospy.Publisher('rfid_readings_list', ReadingList, queue_size=10)

    def send_readings(self, readings):
        """
        publishes a list of readings
        :param readings: list of readings
        :return:
        """
        self.readings_publisher.publish(readings)
