#!/usr/bin/env python

import socket
import sys

# quick hack so server script can be found
sys.path.append('..')
from imud import DEFAULT_PORT

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# change next line as required for your setup.
s.connect(('localhost', DEFAULT_PORT))

while True:
    d = s.recv(4096)
    print d
    # exit if socket closed or we have all data
    if len(d) == 0 or d.endswith('OK\n'): break
s.close()

    
