import rospy
import threading
import helpers
from database_robot.msg import Reading as MessageReading
from std_srvs.srv import Trigger


class Reading:
    '''
    Class to store a reading
    '''
    def __init__(self):
        self.antenna_port = None
        self.ts = None
        self.rf_phase = None
        self.rssi = None
        self.freq = None
        self.mux1 = None
        self.mux2 = None
        self.epc = ""
        self.device_id = ""
        self.robot_pose = list()

    def __eq__(self, other):
        res = True
        #Get simple attributes for both objects
        fields = [a for a in dir(self) if not a.startswith('__') and not callable(getattr(self, a))]
        for i in range(len(fields)):
            res = res and getattr(self, fields[i]) == getattr(other, fields[i])
        return res

    def print_reading(self):
        fields = [a for a in dir(self) if not a.startswith('__') and not callable(getattr(self, a))]
        for i in range(len(fields)):
            print(fields[i])
            print(getattr(self, fields[i]))


class DatabaseReading:
    '''
    Converts a reading into proper database format
    '''
    def __init__(self):
        pass

    @staticmethod
    def convert_readings_database(readings, id_mission, id_section):
        '''
        Converts from a reading object to the proper format for the database
        :param readings: list of objects of type Reading
        :param id_mission: Id of the mission
        :param id_section: Id of the section
        :return:
        '''
        db_readings = list()
        for reading in readings:
            db_reading = MessageReading()
            db_reading.id_reading = str(reading.epc) + "-" + str(reading.ts)
            db_reading.id_mission = str(id_mission)
            db_reading.id_section = str(id_section)
            db_reading.antenna_port = str(reading.antenna_port)
            db_reading.timestamp = str(reading.ts)
            if reading.rf_phase:
                db_reading.rf_phase = str(reading.rf_phase)
            else:
                db_reading.rf_phase = str(0)
            db_reading.rssi = str(reading.rssi)
            db_reading.freq = str(reading.freq)
            db_reading.mux1 = str(reading.mux1)
            db_reading.mux2 = str(reading.mux2)
            db_reading.epc = str(reading.epc)
            db_reading.device_id = str(reading.device_id)
            db_reading.robot_pose = str(reading.robot_pose)
            db_readings.append(db_reading)
        return db_readings


class ReadingQueue:
    '''
    Queue used as buffer to store readings coming from the readers and wait until they are stored in the database.
    It is a shared resource of different threads (each reader is a thread pushing elements plus the thread poping and
    storing them)
    '''
    def __init__(self):
        self.lock = threading.Lock()
        self.queue = list()

    def push(self, reading):
        self.lock.acquire()
        self.queue.append(reading)
        self.lock.release()

    def pop(self):
        self.lock.acquire()
        if self.queue:
            element = self.queue.pop(0)
        else:
            element = None
        self.lock.release()
        return element

    def pop_all(self):
        self.lock.acquire()
        elements = self.queue
        self.queue = list()
        self.lock.release()
        return elements

    def has_items(self):
        if self.queue:
            return True
        else:
            return False


class ManageReadings:
    '''
    Class used to manage readings coming from the readers
    '''
    def __init__(self, reading_queue, id_mission, id_section):
        self.id_mission = id_mission
        self.id_section = id_section
        self.twisting = False
        self.reading_queue = reading_queue
        self.db_reading = DatabaseReading()
        self.publish_readings = helpers.ReadingListPublisher()
        self.store_readings = helpers.StoreReadings()
        # RFID communication service
        rospy.Service('rfid_readings_step_manager', Trigger, self.manage_step_readings)

    def manage_step_readings(self, msg):
        '''
        Manages the readings stored by the readers
        :return:
        '''
        #Get the readings in the QUEUE
        readings = self.reading_queue.pop_all()
        if readings:
            #Convert readings into format
            db_readings = self.db_reading.convert_readings_database(readings, self.id_mission, self.id_section)

            #Store readings
            self.store_readings.store_readings(db_readings)

            # Publish readings
            self.publish_readings.send_readings(db_readings)

            return True, 'step readings stored and sent'
        return False, 'no tags read'
