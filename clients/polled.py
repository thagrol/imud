#!/usr/bin/env python

import socket
import sys
import time

# quick hack so server script can be found
sys.path.append('..')
from imud import LISTEN_PORT

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# change next line as required for your setup.
s.connect(('localhost', LISTEN_PORT))

d=''

while True:
    d += s.recv(4096)
    # exit if socket closed
    if len(d) == 0: break
    if d.endswith('OK\n'):
        print d
        d=''
        print '\t\tSleeping...'
        time.sleep(2)
        print '\t\tPolling...'
        s.sendall('?\n')
s.close()

    
