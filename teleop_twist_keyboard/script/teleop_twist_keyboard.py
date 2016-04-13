#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

u/m : increase/decrease max speeds by 10%
i/, : increase/decrease only linear speed by 10%
o/. : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'w':(1,0),
		'e':(1,-1),
		'a':(0,1),
		'd':(0,-1),
		'q':(1,1),
		'x':(-1,0),
		'c':(-1,1),
		'z':(-1,-1),
		's':(0,0),
	       }

speedBindings={
		'u':(1.1,1.1),
		'm':(.9,.9),
		'i':(1.1,1),
		',':(.9,1),
		'o':(1,1.1),
		'.':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = .5

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	platform_name = 'virtual'
	if rospy.search_param('robot_platform'):
	    platform_name = 'physical'
    	settings = termios.tcgetattr(sys.stdin)
    	print platform_name
	
	cmd_vel_topic = '/hri_robot/cmd_vel'
	if platform_name == 'physical':
	    cmd_vel_topic = '/cmd_vel'
	    
	print cmd_vel_topic
	
	pubms = rospy.Publisher('cmd_motor_state', MotorState, queue_size=10)
	pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
	rospy.init_node('teleop_twist_keyboard')

	ms = MotorState()
	ms = 1
	pubms.publish(ms)
	#k = 0
	#while (k < 1000):
		#k = k + 1
		#pubms.publish(ms)

	x = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			print ms
			pubms.publish(ms)
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
			if platform_name != 'physical':
	                    if turn > 0:
			        turn = -turn
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			print twist
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


