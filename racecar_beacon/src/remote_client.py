#!/usr/bin/env python

import socket
from struct import *
format_RPOS = "fffxxxx" # 3 float32 et 4 octets vides
format_OBSF = "Ixxxxxxxxxxxx" # 1 uint32 et 12 octets vides
format_RBID = "Ixxxxxxxxxxxx" # 1 uint32 et 12 octets vides

HOST = '192.168.0.1'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65432

s= socket(AF_INET, SOCK_STREAM)
s.connect((HOST, PORT)) # connect to server (block until accepted)
while True:
	sent=raw_input(“>”)
	if sent like "RPOS":
		s.send(sent) # send same data
		data = s.recv(16) # receive the response
		data_decode=unpack(format_RPOS,data)
	elif sent like "OBSF":
		s.send(sent) # send same data
		data = s.recv(16) # receive the response
		data_decode=unpack(format_OBSF,data)
	elif sent like "RBID":
		s.send(sent) # send same data
		data = s.recv(16) # receive the response
		data_decode=unpack(format_RBID,data)
	else:
		print("This command is not specified")
	print data_decode # print the result
s.close() # close the connection

