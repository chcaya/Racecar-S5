#!/usr/bin/env python3

import socket
from struct import *

from racecar_beacon_constants import *


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Socket UDP
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) # Mode broadcast
s.bind(("255.255.255.255", VEHICLE_TRACKER_PORT))

data, addr = s.recvfrom(16) # receive the response
(x, y, theta, id) = unpack(FORMAT_POSBROAD, data)
print(str(x) + ", " + str(y) + ", " +str(theta) + ", " +str(id))
