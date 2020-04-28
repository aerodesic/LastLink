#!/usr/bin/env python3
import sys

print("Using '%s'" % sys.argv[1], file=sys.stderr)

with open(sys.argv[1], "w") as f:
    print('config ABC_DEF', file=f)
    print('  bool "abc"', file=f)

