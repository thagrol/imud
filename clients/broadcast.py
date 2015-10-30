#!/usr/bin/env python

import socket
import sys

# quick hack so server script can be found
sys.path.append('..')
from imud import DEFAULT_PORT

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('<broadcast>', DEFAULT_PORT))

d=''
update_interval_set = False

while True:
    d += s.recv(4096)
    # exit if socket closed
    if len(d) == 0: break
    if d.endswith('OK\n'):
        print d,
        d=''
s.close()

    
