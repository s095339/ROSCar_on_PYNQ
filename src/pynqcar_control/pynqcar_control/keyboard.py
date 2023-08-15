#!/usr/bin/env python
import rclpy
from rclpy.qos import qos_profile_default

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import numpy as np
import time

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d

Diagonal movement:
	7	 	9

	1		3

command mode: /

Rotation:
q/e: counter-clock-wise/clock-wise

z/x : increase/decrease max speeds by 10%

anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'w':(1,0,0,0),
		'a':(0,1,0,0),
		's':(-1,0,0,0),
		'd':(0,-1,0,0),
		'q':(0,0,0,1),
		'e':(0,0,0,-1),
		'9':(1,-1,0,0),
		'7':(1,1,0,0),
		'1':(-1,1,0,0),
		'3':(-1,-1,0,0)
	       }

speedBindings={
		'z':(1.1,1.1),
		'x':(.9,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	
	if args is None:
		args = sys.argv

	rclpy.init(args=args)
	node = rclpy.create_node('teleop_twist_keyboard')
		
	pub = node.create_publisher(Twist, 'cmd_vel', 	qos_profile_default)
	max_speed = 0.12
	max_turn = 0.7
	speed = 0.06
	turn = 0.35
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			print('pressed: ', key)
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				speed = np.clip(speed, -max_speed, max_speed)
				turn = np.clip(turn, -max_turn, max_turn)

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				if key == '/': # forward for 10 s
					cmd = input('/')
					if cmd in ['f', 'forward']:
						x = 1
						y = 0
						z = 0
						th = 0
					elif cmd in ['b', 'backward']:
						x = -1
						y = 0
						z = 0
						th = 0
					else:
						print('command not found')
						x = 0
						y = 0
						z = 0
						th = 0
					twist = Twist()
					twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
					twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
					pub.publish(twist)
					time.sleep(10)
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print('error!')

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)