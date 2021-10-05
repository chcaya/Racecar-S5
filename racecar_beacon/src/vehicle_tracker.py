#!/usr/bin/env python

import socket
from struct import *
format_PosBroad = "fffI" # 3 float32 et 1 uint32

HOST = '127.0.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

s= socket(AF_INET, SOCK_DGRAM)
s.bind((HOST, PORT))

while True:
	(data, addr) = sock.recvfrom(16)
	decode_data=unpack(format_PosBroad,data)
	print("address",addr)
	print("msg",decode)
