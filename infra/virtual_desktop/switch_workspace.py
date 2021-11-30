#!/usr/bin/python3

# "Helper for setting current desktop's name"
# pip install xcffib xpybutil

import sys

from xpybutil import conn, root
import xpybutil.ewmh as ewmh
if len(sys.argv) != 2:
    print("Invalid argument")
    sys.exit(1)
num = int(sys.argv[1])
print("Switching to ws", num)
ewmh.request_current_desktop_checked(num).check()
