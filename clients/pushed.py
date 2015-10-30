#!/usr/bin/env python

import socket
import sys

# quick hack so server script can be found
sys.path.append('..')
from imud import DEFAULT_PORT

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# change next line as required for your setup.
s.connect(('localhost', DEFAULT_PORT))

d=''
update_interval_set = False

while True:
    d += s.recv(4096)
    # exit if socket closed
    if len(d) == 0: break
    if d.endswith('OK\n'):
        print d,
        d=''
        if not update_interval_set:
            print '\t\tSetting auto update interval to 2 seconds'
            s.sendall('u=2\n')
            update_interval_set = True
s.close()

    
