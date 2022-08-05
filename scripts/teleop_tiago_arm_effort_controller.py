#!/usr/bin/env python

import sys
import getch
import rospy
from tiago_arm_effort_controller.msg import Teleop

TeleopMsg = Teleop()
TeleopMsg.control_type = Teleop.GC
TeleopMsg.coeff_cmd = 1.


msg = """
Reading from the keyboard and Publishing to /teleop
---------------------------
Switching between types of controllers (control_type):
    a:  gravity compensation
    z:  pose control on q
    e:  pose control on X
    r:  dual control (pose & force)

Increasing/decreasing Kp coefficient (coeff_cmd):
    j:  decrease 
    k:  default (value 1)
    l:  increase

CTRL-C to quit
"""

def keys():
    pub = rospy.Publisher('my_tiago_controller/teleop', Teleop,queue_size=1)
    rospy.init_node('keypress',anonymous=True)
    rate = rospy.Rate(10)#try removing this line and see what happens
    while not rospy.is_shutdown():
        k=ord(getch.getch())# this is used to convert the keypress event in the keyboard or joypad , joystick to a ord value
        if   k ==  97: #'a'
            TeleopMsg.control_type=Teleop.GC
        elif k == 122: #'z'
            TeleopMsg.control_type=Teleop.PC_q
        elif k == 101: #'e'
            TeleopMsg.control_type=Teleop.PC_X
        elif k == 114: #'r'
            TeleopMsg.control_type=Teleop.DC

        elif k == 111: #'o'
            TeleopMsg.control_lib=Teleop.RBDL
        elif k == 112: #'p'
            TeleopMsg.control_lib=Teleop.Pin

        elif k == 106: #'j'
            TeleopMsg.coeff_cmd=TeleopMsg.coeff_cmd/2
        elif k == 107: #'k'
            TeleopMsg.coeff_cmd=1.
        elif k == 108: #'l'
            TeleopMsg.coeff_cmd=TeleopMsg.coeff_cmd*2

        pub.publish(TeleopMsg)#to publish
        rospy.loginfo(str(k))

        #rate.sleep()

#s=115,e=101,g=103,b=98

if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass
