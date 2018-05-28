#!/usr/bin/python

import csv
import sys

with open(sys.argv[1], 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    next(reader, None)  # Skip the header

    print " [",
    for row in reader:
        print '0x{:02X},'.format(int(row[3],16)),

    print "]"

