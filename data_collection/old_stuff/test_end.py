import time
import os
import sys


LINE_UP = '\033[1A'
LINE_CLEAR = '\x1b[2K'

p = "11"
for i in range(10, 0, -1):
#    os.system('cls' if os.name == 'nt' else 'clear')
#    print(chr(27) + "[2J")
#    os.system('cls||clear')
#    print("\033c", end="")
#    sys.stdout.flush()
    for j in range(10-i):
        print(LINE_UP, end=LINE_CLEAR)
    print(p)
    p = str(i) + "\n" + p
    time.sleep(.5)
print()
