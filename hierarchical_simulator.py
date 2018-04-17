from queue import Queue
from threading import Thread
import time


# num_agents = 5
#
# def compute_e_score():
#     m = 5
#
# for i in range(num_agents)
#      t = Thread(target=compute_e_score)
#      t.daemon = True
#      t.start()

import threading
import urllib.request
import urllib.parse
import json
import time


class Device(threading.Thread):
    def __init__(self, name, endpoint, preamble = 'state', port = 80 ):
        threading.Thread.__init__(self)
        self.name = name
        self.connected = False
        self.connection = HTTPConnection(endpoint, preamble, port)
        print('%s: __init__' % self.name)

    def run(self):
        self.getStatus()
        print('%s: hit run()' % self.name)

    def getStatus(self):
        self.urlresponse = json.loads(self.connection.GET('get/USA/all')) #Use USA just to verify connection
        self.connected = True

    def checkcountry(self):
        if (self.name == 'USA'): self.waittime = 10
        else: self.waittime = 0

        print('%s: Getting Codes - wait time: %s' % (self.name, self.waittime))

        start_time=time.time()
        time.sleep(self.waittime)
        result =self.connection.GET('get/%s/all' % self.name)
        elapsed_time=time.time() - start_time
        print('%s: Got Codes - second: %s' % (self.name, elapsed_time))


class HTTPConnection:
    def __init__(self, endpoint, preamble = None, port = 80):
        if preamble: # specificing a version after the port and before method
            self.url = 'http://%s:%s/%s/' % (endpoint, port, preamble)
        else:
            self.url = 'http://%s:%s/' % (endpoint, port)

    def GET(self, operation):
        with urllib.request.urlopen('%s%s' % (self.url, operation)) as f:
             return f.read().decode('utf-8')


DeviceList = {'USA':'services.groupkt.com', 'IND':'services.groupkt.com'}
ActiveDevices = []

DeviceList = {'USA':'services.groupkt.com', 'IND':'services.groupkt.com'}
ActiveDevices = []

for name, ip in DeviceList.items():
    print('main: creating object for: %s' % name)
    newDevice = Device(name, ip)
    ActiveDevices.append(newDevice)
    newDevice.start()

for device in ActiveDevices:
    print('main: calling checkcountry() for: %s' % device.name)
    device.checkcountry()