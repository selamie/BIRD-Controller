#!/usr/bin/env python
import rospy
from apc_arduino.msg import HallEffectData
from std_msgs.msg import Float32

now = 0.0
last_time = 0.0
first_time = True
tick = 0.0
last_tick = 0.1

def topiccallback(data):
    global now,last_time,first_time, pub
    now = rospy.get_time()
    if first_time:
        first_time = False
        last_time = now
    if now > last_time + .2:
        rospy.logwarn("Arduino node frequency lower than 5 hz")
    if not now == last_time:
        pub.publish(1.0/(now - last_time))
    last_time = now
    
def timercallback(event):
    global last_tick
    tick = now
    if tick == last_tick and not first_time:
        rospy.logwarn("Arduino node didn't publish for 2 seconds. If this only happens once, don't worry.")
    last_tick = tick

def listener():
    global pub
    
    rospy.init_node('Connection_Checker', anonymous=True)
    
    pub = rospy.Publisher('/Arduino_Frequency', Float32, queue_size=10)

    rospy.Subscriber("/Arduino/HEData", HallEffectData, topiccallback)
    
    timer = rospy.Timer(rospy.Duration(2),timercallback) 
    
    rospy.spin()
    
if __name__ == '__main__':
    listener()
