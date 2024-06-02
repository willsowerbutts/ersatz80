#!/usr/bin/env python3

import sys

output = open(sys.argv[1], 'w')
data = open(sys.argv[2], 'rb').read()

output.write("#include \"rom.h\"\n")
output.write("const uint8_t monitor_rom[] = {\n")

while data:
    output.write(", ".join('0x%02x' % x for x in data[:16]) + ',\n')
    data = data[16:]

output.write("};\n")
