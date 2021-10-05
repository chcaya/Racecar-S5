#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
sent_data = ""
while sent_data is not "stop":
	sent_data = raw_input(">")
	s.send(sent_data)
	data=s.recv(1024)
	print data
s.close()


