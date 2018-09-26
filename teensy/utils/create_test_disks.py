#!/usr/bin/env python

import sys

output = open(sys.argv[1], 'w')

for x in range(0,256):
    for y in range(0,256):
        for z in range (0,256):
            output.write(bytearray([x, y, z]))
            
output = open(sys.argv[2], 'w')

for x in range(0,256):
    for y in range(0,256):
        output.write(bytearray([x, y]))

output = open(sys.argv[3], 'w')

for x in range(0,256):
    for y in range(0,256):
        for z in range (0,256):
            output.write(bytearray([0, 0, 0]))
            
output = open(sys.argv[4], 'w')

for x in range(0,256):
    for y in range(0,256):
        output.write(bytearray([0, 0]))
