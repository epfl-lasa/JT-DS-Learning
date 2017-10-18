#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from kuka_fri_bridge import JointStateImpedance

q_initial = [-0.728371429443359,-1.3605418920517,2.69252680319093,0.620675325393677,0.955035626888275,0.141930669546127,-0.17979271709919]
x_end = [-.5, -.5, 0.3]

def listener_callback(data):

    q_current = get_q_from_data(data)



def talker():
    pub = rospy.Publisher('/KUKA/joint_imp_cmd', JointStateImpedance, queue_size=50)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(100) # 100hz


    # Index of statuses:
    # 0 - go to initial joint position
    # 1 - do learned move to task target
    status = 0 


    while not rospy.is_shutdown():





        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass