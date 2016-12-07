#!/usr/bin/python
from std_msgs.msg import String
import rospy


class Listener:
    def __init__(self):
        self.ok = False
        self.pub = rospy.Publisher('/token_manager/current', String, queue_size=10)
        self.current = String()
        self.t0 = 0
        
        rospy.Subscriber('/token_manager/requested', String, self.readToken)
        
        
    def update(self):
        self.pub.publish(self.current)
        
        
    def readToken(self, msg):
        self.ok = True
        
        
        
        t = rospy.Time.now().to_sec()
        
        if self.current.data in ('', msg.data):
            print msg.data, 'is still on Baxter'
            # store last time this was received
            self.t0 = t
            self.current.data = msg.data
        elif self.current.data != msg.data and t - self.t0 > 1:
            print self.current.data, 'was just replaced by', msg.data
            # other request while initial was not done for 1 sec -> change
            self.t0 = t
            self.current.data = msg.data

if __name__ == '__main__':
    '''
    Begin of main code
    '''                    
    rospy.init_node('token_manager')
    
    listener = Listener()
    
    while not rospy.is_shutdown():
        
        if listener.ok:
            listener.update()
        
        rospy.sleep(0.1)
    
