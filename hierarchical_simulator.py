import threading

import json
import time


class e_agent(threading.Thread):
    def __init__(self, name, is_leader, id):
        threading.Thread.__init__(self)
        self.name = name

        self.leader_status = is_leader
        self.value = 5
        print('%s: __init__' % self.name)

    def run(self):
        self.send_value()


    def send_value(self):
        print('%s: Matrix Sent' % self.name)
        # Write something that makes this wait for all clear



# class HTTPConnection:
#     def __init__(self, endpoint, preamble = None, port = 80):
#         if preamble: # specificing a version after the port and before method
#             self.url = 'http://%s:%s/%s/' % (endpoint, port, preamble)
#         else:
#             self.url = 'http://%s:%s/' % (endpoint, port)
#
#     def GET(self, operation):
#         with urllib.request.urlopen('%s%s' % (self.url, operation)) as f:
#              return f.read().decode('utf-8')





DeviceList = {'USA':'services.groupkt.com', 'IND':'services.groupkt.com'}
ActiveDevices = []


for name, ip in DeviceList.items():
    print('main: creating object for: %s' % name)
    newDevice = e_agent(name, ip)
    ActiveDevices.append(newDevice)
    newDevice.start()

for device in ActiveDevices:
    print('main: calling checkcountry() for: %s' % device.name)
    device.checkcountry()