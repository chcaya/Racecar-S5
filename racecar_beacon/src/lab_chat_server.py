#!/usr/bin/env python

import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
(conn,addr)= s.accept()
while True:
	data=conn.recv(1024)
	if not data: break
	print data
	conn.send(raw_input(">"))
conn.close()
