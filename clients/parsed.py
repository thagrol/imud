#!/usr/bin/env python

import ast
import socket
import sys

# quick hack so server script can be found
sys.path.append('..')
from imud import LISTEN_PORT

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# change next line as required for your setup.
s.connect(('localhost', LISTEN_PORT))

d = ''

while True:
    n = s.recv(4096)
    d += n
    # exit if socket closed or we have all data
    if len(n) == 0 or d.endswith('OK\n'): break
s.close()

# parse received data
for l in d.splitlines():
    if l == 'hello':
        pass
    elif l.startswith('v'):
        v = ast.literal_eval(l[1:])
    elif l.startswith('available:'):
        a = l[10:]
    elif l.startswith('o:'):
        o = ast.literal_eval(l[2:])
    elif l.startswith('h:'):
        h = ast.literal_eval(l[2:])
    elif l.startswith('t:'):
        t = ast.literal_eval(l[2:])
    elif l.startswith('p:'):
        p = ast.literal_eval(l[2:])
    elif l.startswith('i:'):
        i = bool(ast.literal_eval(l[2:]))
    elif l.startswith('u:'):
        u = ast.literal_eval(l[2:])
    elif l == 'OK':
        pass
    else:
        # should never get here
        pass

print 'Protocol version:\t%s' % v
print 'Available data:\t\t%s' % a
print 'Orentation:'
print '\tx:\t\t%f' % o['x']
print '\ty:\t\t%f' % o['y']
print '\tz:\t\t%f' % o['z']
print 'Temperature:\t\t%fC' % t
print 'Pressure:\t\t%f' % p
print 'Inverted:\t\t%s' % i
print 'Update interval:\t%f' % u
print 'Heading:\t\t%s' % h[0]
print '\t\t\t%s' % h[1]
