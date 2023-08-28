#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import numpy as np
import time
import os
from pathlib import Path
from nav_msgs.msg import Odometry
import math

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

class KeyBoard(Node):
	def __init__(self):
		super().__init__('keyboard')
		self.subscription = self.create_subscription(
            Odometry,
            'wheel_odom',
            self.wheel_odom_CB,
            10)
		self.subscription  # prevent unused variable warning
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 	qos_profile_default)

		# init variables
		self.x_now = 0.0  # 
		self.y_now = 0.0  # current orientation
		self.th_now = 0.0 # 
		self.get_logger().info('Keyboard start')
		self.get_logger().info('Subscribed /wheel_odom(Odometry)')

	def start(self):
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
			print(self.vels(speed,turn))
			while(1):
				key = self.getKey()
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

					print(self.vels(speed,turn))
					if (status == 14):
						print(msg)
					status = (status + 1) % 15
				elif key == '/': # command input
					cmds = input('/').split()
					try:
						cmd = cmds[0]
						length = cmds[1]
						unit = cmds[2]
					except:
						pass
						
					if cmd == 'help':
						print("commands:")
						print("  move <direction> for <length> <unit>: <direction> <length> <unit>")
						print("    <direction> = w, s")
						print("    unit = m(meters)")
						print("  rotate for <length> <unit>: <direction> <length> <unit>")
						print("    <direction>: q, e")
						print("    unit = m(meters)")
					elif cmd in ['w', 's']:
						rclpy.spin_once(self)
						x_start = self.x_now

						diff = 0.0
						x, y, z, th = moveBindings[cmd]
						self.publisher_.publish(self.twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
						if unit == 'm':
							length = float(length)
						else:
							length = float(length)
							print('no such unit, set to default: meter')
						while -length <= diff <= length: # meters
								try: # for emergency break: crtl-c
									rclpy.spin_once(self)
									diff = self.x_now - x_start
								except:
									self.publisher_.publish(self.twist_val(x = 0, y = 0, z = 0, th = 0, speed = speed, turn = turn))
						x = 0.0
						y = 0.0
						z = 0.0
						th = 0.0
						print('done, difference between target: {} {}'.format(abs(diff) - length, unit))
						self.publisher_.publish(self.twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
						
					elif cmd in ['q', 'e']: # rotation
						rclpy.spin_once(self)
						th_start = self.th_now
						diff = 0.0
						x, y, z, th = moveBindings[cmd]
						self.publisher_.publish(self.twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
						if unit == 'd':
							length = float(length) / 180 * math.pi
						elif unit == 'r': # radian
							length = float(length) 
						else:
							length = float(length) / 180 * math.pi
							print('no such unit, set to default unit: degree')
						while -length <= diff <= length: # meters
								try: # for emergency break: crtl-c
									rclpy.spin_once(self)
									diff = self.th_now - th_start
								except:
									self.publisher_.publish(self.twist_val(x = 0, y = 0, z = 0, th = 0, speed = speed, turn = turn))
									break
						x = 0.0
						y = 0.0
						z = 0.0
						th = 0.0
						print('done, difference between target: {} degrees'.format((abs(diff) - length) / math.pi * 180))
						self.publisher_.publish(self.twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
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
				else:
					x = 0
					y = 0
					z = 0
					th = 0
					if (key == '\x03'):
						break
				self.publisher_.publish(self.twist_val(x = x, y = y, z = z, th = th, speed= speed, turn = turn))
				time.sleep(0.01)

		finally:
			self.publisher_.publish(self.twist_val())
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

		

	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key


	def vels(self, speed,turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)

	def uint32toint(self, value):
		if (value>= 2**31): # negative value
			value-= 2**32
		return value

	def twist_val(self, x = 0.0, y = 0.0, z = 0.0, th = 0.0, speed = 0.0, turn = 0.0):
		twist = Twist()
		twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
		return twist

	def wheel_odom_CB(self, msg):
		self.x_now  =msg.pose.pose.position.x
		self.y_now = msg.pose.pose.position.y
		_, self.th_now, _ = self.ToEulerAngles(msg.pose.pose.orientation)

	def ToEulerAngles(self, q):
		# roll (x-axis rotation)
		sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
		cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
		roll = math.atan2(sinr_cosp, cosr_cosp)

		# pitch (y-axis rotation)
		sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
		cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
		pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

		# yaw (z-axis rotation)
		siny_cosp = 2 * (q.w * q.z + q.x * q.y)
		cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
		yaw = math.atan2(siny_cosp, cosy_cosp)
		return roll, yaw, pitch



def main(args=None):	
	if args is None:
		args = sys.argv

	rclpy.init(args=args)
	keyboard = KeyBoard()
	keyboard.start()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	keyboard.destroy_node()
	rclpy.shutdown()

	