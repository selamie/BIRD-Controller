#!/usr/bin/env python
import rospy
from apc_arduino.msg import Spatula_Position
from sensor_msgs.msg import JointState


joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)


_param1 = rospy.get_param("/arduino_params/Spatula_F1_Zero",3000.0)
_param2 = rospy.get_param("/arduino_params/Spatula_F2_Zero",3000.0)
_seq = 0

def callback(data):
    global _seq
    pos = [data.Sp1,data.Sp2]
    
    jnames = ['joint_nail_left', 'joint_nail_right']
    
    js = JointState()
    js.header.seq = _seq
    _seq += 1
    js.header.stamp = rospy.Time.now()
    js.name  = jnames
    
    js.position = [pos[0] / float(_param1) * 3.14159265359, pos[1] / float(_param2) * 3.14159265359]
    joint_pub.publish(js)
    

if __name__=='__main__':
    rospy.init_node('spatula_joint_publisher', anonymous=True)
    sub = rospy.Subscriber('/Arduino/Spatula_pos', Spatula_Position, callback)
    rospy.spin()
