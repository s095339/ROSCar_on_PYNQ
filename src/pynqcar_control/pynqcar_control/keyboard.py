#!/usr/bin/env python
import rclpy
from rclpy.qos import qos_profile_default

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import numpy as np
import time
import os
from pathlib import Path

for i in range(10):
	workspace = Path(__file__).parents[i] # __file__ is location of current file in install
	if str(workspace).split('/')[-1] == 'ROSCar_on_PYNQ': # detect folder name
		break
print("workspace = ",workspace)
sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')

from pynq import Overlay

settings = termios.tcgetattr(sys.stdin)
# Car spec, assume car goes in x direction
lx = 0.075 # meters
ly = 0.0975 # meters
r = 0.03025477707 # meters

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

def uint32toint(value):
	if (value>= 2**31): # negative value
		value-= 2**32
	return value

def twist_val(x = 0.0, y = 0.0, z = 0.0, th = 0.0, speed = 0.0, turn = 0.0):
	twist = Twist()
	twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
	twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
	return twist


def main(args=None):	
	if args is None:
		args = sys.argv

	print('loading overlay woth download = False')
	ov_pth = os.path.join(workspace, 'hardware', 'uart.bit')
	overlay = Overlay(ov_pth, download = False)
	print('done')

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
					start_M1 = uint32toint(overlay.encoder_0.read(0x00)) # wheel rear left [encoder ticks]
					start_M2 = uint32toint(overlay.encoder_0.read(0x04)) # wheel front left [encoder ticks]
					start_M3 = uint32toint(overlay.encoder_0.read(0x08)) # wheel front right [encoder ticks]
					start_M4 = uint32toint(overlay.encoder_0.read(0x0c)) # wheel rear right [encoder ticks]
					
					D1 = 0.0
					D2 = 0.0
					D3 = 0.0
					D4 = 0.0

					x, y, z, th = moveBindings[cmd]
					pub.publish(twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
					if unit == 'm':
						while -float(length) <= (D1 + D2 + D3+ D4) / 4 <= float(length): # meters
							try: # for emergency break: crtl-c
								M1 = uint32toint(overlay.encoder_0.read(0x00)) # wheel rear left [encoder ticks]
								M2 = uint32toint(overlay.encoder_0.read(0x04)) # wheel front left [encoder ticks]
								M3 = uint32toint(overlay.encoder_0.read(0x08)) # wheel front right [encoder ticks]
								M4 = uint32toint(overlay.encoder_0.read(0x0c)) # wheel rear right [encoder ticks]
								D1 = (M1 - start_M1) / 4317.4 * 0.19 # [m], 4317.4 = encoder/revolution, 0.19 m/revolution
								D2 = (M2 - start_M2) / 4323.3 * 0.19 # [m]
								D3 = (M3 - start_M3) / 4317.3 * 0.19 # [m]
								D4 = (M4 - start_M4) / 4310.0 * 0.19 # [m]
								time.sleep(0.01)
							except:
								pub.publish(twist_val(x = 0, y = 0, z = 0, th = 0, speed = speed, turn = turn))
					else:
						print('no such unit: ', unit)
					x = 0.0
					y = 0.0
					z = 0.0
					th = 0.0
					print('done')
					pub.publish(twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
					
				elif 'q' in cmd or 'e' in cmd: # rotation
					start_M1 = uint32toint(overlay.encoder_0.read(0x00)) # wheel rear left [encoder ticks]
					start_M2 = uint32toint(overlay.encoder_0.read(0x04)) # wheel front left [encoder ticks]
					start_M3 = uint32toint(overlay.encoder_0.read(0x08)) # wheel front right [encoder ticks]
					start_M4 = uint32toint(overlay.encoder_0.read(0x0c)) # wheel rear right [encoder ticks]
					
					D1 = 0.0
					D2 = 0.0
					D3 = 0.0
					D4 = 0.0
					x, y, z, th = moveBindings[cmd]
					pub.publish(twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
					if unit == 'd':
						while -float(length) <= (-D1 - D2 + D3+ D4) / 4 / (lx + ly) * r <= float(length): # meters
							try: # for emergency break: crtl-c
								M1 = uint32toint(overlay.encoder_0.read(0x00)) # wheel rear left [encoder ticks]
								M2 = uint32toint(overlay.encoder_0.read(0x04)) # wheel front left [encoder ticks]
								M3 = uint32toint(overlay.encoder_0.read(0x08)) # wheel front right [encoder ticks]
								M4 = uint32toint(overlay.encoder_0.read(0x0c)) # wheel rear right [encoder ticks]
								D1 = (M1 - start_M1) / 4317.4 * 360 # [degree], 4317.4 = encoder/revolution,
								D2 = (M2 - start_M2) / 4323.3 * 360 # [degree]
								D3 = (M3 - start_M3) / 4317.3 * 360 # [degree]
								D4 = (M4 - start_M4) / 4310.0 * 360 # [degree]
							except:
								pub.publish(twist_val(x = 0, y = 0, z = 0, th = 0, speed = speed, turn = turn))
								break
					else:
						print('no such unit: ', unit)
					x = 0.0
					y = 0.0
					z = 0.0
					th = 0.0
					print('done')
					pub.publish(twist_val(x = x, y = y, z = z, th = th, speed = speed, turn = turn))
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
			pub.publish(twist_val(x = x, y = y, z = z, th = th, speed= speed, turn = turn))

	finally:
		pub.publish()
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)