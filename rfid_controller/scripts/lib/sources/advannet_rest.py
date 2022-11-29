import urllib2
from xml.etree import cElementTree
#from orderedset import *

def response_reply(r):
    response_body = r
    response_root = cElementTree.fromstring(response_body)
    response_status = response_root.find('status').text
    return response_status

def validate_reader(device_url):
    try:
        request = urllib2.Request(device_url + '/validate')
        r = urllib2.urlopen(request, timeout=60).read()
        response_status = response_reply(r)
        return response_status
    except urllib2.URLError:
        return False

def start_reader(device_url):
    try:
        request = urllib2.Request(device_url + '/start')
        r = urllib2.urlopen(request, timeout=60).read()
        response_status = response_reply(r)
        return response_status
    except urllib2.URLError:
        return False

def set_rf_read_power(device_url, power):
    opener = urllib2.build_opener(urllib2.HTTPHandler)
    request = urllib2.Request(device_url + '/reader/parameter/RF_READ_POWER/', data=str(power))
    request.add_header('Content-Type', 'text/html')
    request.get_method = lambda: 'PUT'
    response = opener.open(request)
    response_status = response_reply(response.read())
    return response_status


def set_session(device_url, session):
    opener = urllib2.build_opener(urllib2.HTTPHandler)
    request = urllib2.Request(device_url + '/reader/parameter/GEN2_SESSION/', data=str(session))
    request.add_header('Content-Type', 'text/html')
    request.get_method = lambda: 'PUT'
    response = opener.open(request)
    response_status = response_reply(response.read())
    return response_status


def stop_reader(device_url):
    request = urllib2.Request(device_url + '/stop')
    r = urllib2.urlopen(request).read()
    response_status = response_reply(r)
    if response_status == 'OK':
        return True
    else:
        return False


def cycle_inventory(device_url, timeout=60, detailed=False):
    request = urllib2.Request(device_url + '/inventory')
    try:
        connection = urllib2.urlopen(request, timeout=timeout)
        inventory = connection.read()
        connection.close()
        return inventory
    except Exception as e:
        raise


def get_reader_conf(device_url):
    try:
        request = urllib2.Request(device_url)
        r = urllib2.urlopen(request, timeout=10).read()
        response_status = response_reply(r)
        return response_status
    except urllib2.URLError:
        return False

def set_read_time(device_url, read_time):
    opener = urllib2.build_opener(urllib2.HTTPHandler)
    req = '<request>' \
              '<class>READMODE_SEQUENTIAL</class>' \
              '<name>SEQUENTIAL</name>' \
              '<readTime>' \
              + str(read_time) + \
              '</readTime>' + \
          '</request>'
    request = urllib2.Request(device_url + '/readModes[0]/', data=req)
    request.add_header('Content-Type', 'text/html')
    request.get_method = lambda: 'PUT'
    url = opener.open(request)


###########################################################################

def get_epc_set_from_inventory_xml(inventory):
    response_root = cElementTree.fromstring(inventory)
    device = None
    for device_id in response_root.iter('deviceId'):
        device = device_id.text
        break
    epc_set = set()
    for item in response_root.iter('item'):
        epc = item.find('epc').text
        epc_set.add(epc)

    return (device, epc_set)


def get_epc_instances_from_inventory_xml(inventory):

    epc_set = set()
    epc_instances = list()

    response_root = cElementTree.fromstring(inventory)

    device_id = response_root.findtext('.//deviceId')
    ts = response_root.findtext('.//ts')

    for item in response_root.iter('item'):
        read = {}
        read['device_id'] = device_id
        epc = item.findtext('epc')
        read['epc'] = epc

        epc_set.add(read['epc'])

        props = {}
        for prop in item.iter('prop'):
            param = prop.text.split(':')[0]
            values = prop.text.split(':')[1].rstrip(',').split(',')
            # TODO Add the rest of instances, not only the first comma separated!
            props[param] = values

        number_of_values = len(values)
        for i in range(number_of_values):
            for key in props.keys():
                read[key] = props[key][i]

            epc_instances.append(read)
            read = {}
            read['device_id'] = device_id
            read['epc'] = epc

    return {'inventory_set': {'device_id': device_id, 'inventory_ts': float(ts)/1000, 'epc_set': epc_set},
            'inventory_instances': epc_instances}
