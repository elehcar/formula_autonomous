#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import TwoFloat
from sense_hat import SenseHat

# nodo che pubblica sul topic "magnetometer_topic" un valore float che rappresenta la rotazione rispetto al Nord geomagnetico
class SenseNode:
	def __init__(self):
		self.node_rate = 10
		self.sense = SenseHat()
		self.pub = rospy.Publisher('magnetometer_topic', TwoFloat, queue_size=10)
	# funzione che usa il metodo get_compass della classe SenseHat che restituisce i gradi rispetto al Nord magnetico
	def sensing(self):
                self.sense.set_imu_config(True, True, True)
                o = self.sense.get_orientation()
		a = self.sense.get_accelerometer_raw()
                north = o["yaw"]
                x = a["x"]
                y = a["y"]
                z = a["z"]msg = TwoFloat()
                # north = self.sense.get_compass()
                msg.left_us = north
                if x < 1 and y <1 and z < 1:
                    msg.right_us = 0.0
                else:
                    msg.right_us = 1.0
		print("{SENSE_HAT} North: " + str(north))


if __name__ == '__main__':
	sense_hat = SenseNode()
	rospy.init_node("sense_hat", anonymous=True)
	loop = rospy.Rate(sense_hat.node_rate)
	while not rospy.is_shutdown():
		sense_hat.sensing()
		loop.sleep()
