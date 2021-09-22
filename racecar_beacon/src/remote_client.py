#!/usr/bin/env python3

import sys

import socket
from struct import *
from time import thread_time

from racecar_beacon_constants import *


def data_request(socket, cmd):
    socket.send(cmd) # send same data
    data = socket.recv(16) # receive the response

    return data


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, REMOTE_CLIENT_PORT))

arg = sys.argv[1]
arg = arg.upper()
cmd = bytes(arg, "utf-8")

if arg == "RPOS":
    (x, y, theta) = unpack(FORMAT_RPOS, data_request(s, cmd))
    print(str(x) + ", " + str(y) + ", " +str(theta))

elif arg == "OBSF":
    is_obstacle = unpack(FORMAT_OBSF, data_request(s, cmd))[0]
    print(str(is_obstacle))

elif arg == "RBID":
    id = unpack(FORMAT_RBID, data_request(s, cmd))[0]
    print(str(id))

s.close() # close the connection
