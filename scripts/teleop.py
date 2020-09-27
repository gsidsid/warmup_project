from janitor import Janitor

import tty
import select
import sys
import termios

import rospy

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None

rospy.init_node('teleop_twist_keyboard')

neato = Janitor()
neato.connect()
neato.drive(0,0)

while key != '\x03':
    key = getKey()
    if key is 'w':
        print(key)
        neato.drive(1,0)
    elif key is 'a':
        neato.drive(0,-1)
    elif key is 'd':
        neato.drive(0,1)
    else:
        neato.drive(0,0)

neato._stop()
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)