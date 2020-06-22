#!/usr/bin/python

import sys
from intelhex import IntelHex

print('Number of arguments:', len(sys.argv), 'arguments.')
print('Argument List:', str(sys.argv))
if(len(sys.argv) > 2):
    print('Too many arguments, use this instead: python dump_hex.py <hex_file>.hex')
    exit(-1);

hex_file = IntelHex()
hex_file.loadfile(sys.argv[1], format='hex')

file = open((str(sys.argv[1]) + ".txt"), 'w')

print('Dumping hex file contents into: ' + (str(sys.argv[1]) + ".txt"))
hex_file.dump(file)

file.close()
