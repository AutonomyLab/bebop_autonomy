#!/usr/bin/python

"""Listen to joy autoflight node and publish to complex autoflight topics."""

import rospy
from std_msgs.msg import Bool, Empty, String


class AutoFlightJoy:
    """Subscribe to joy autoflight node and publish to bebop driver node."""

    navigatehome = True
    usertakeoff = True

    def __init__(self):
        """Initialize publishers and subscribers."""
        self.pub_start = rospy.Publisher('autoflight/start', String,
                                         queue_size=1)
        self.pub_home = rospy.Publisher('autoflight/navigate_home', Bool,
                                        queue_size=1)
        self.pub_usertakeoff = rospy.Publisher('user_takeoff', Bool,
                                               queue_size=1)
        self.pub_record = rospy.Publisher('record', Bool, queue_size=1)

        rospy.Subscriber('autoflight/start/joy', Empty, self.start_callback)
        rospy.Subscriber('autoflight/navigate_home/joy', Empty,
                         self.home_callback)
        rospy.Subscriber('user_takeoff/joy', Empty, self.usertakeoff_callback)
        rospy.Subscriber('record/joy/start', Empty, self.record_start_callback)
        rospy.Subscriber('record/joy/stop', Empty, self.record_stop_callback)

        rospy.loginfo('autoflight: Ready to receive commands from joy.')

    def start_callback(self, empty):
        """Retrieve flight plan name and publish to bebop autoflight/start."""
        string = rospy.get_param('~autoflight/start/fname',
                                 'flightPlan.mavlink')
        self.pub_start.publish(string)

    def home_callback(self, empty):
        """Publish alternating bool to bebop autoflight/navigate_home."""
        if self.navigatehome:
            self.pub_home.publish(self.navigatehome)
            self.navigatehome = False
        else:
            self.pub_home.publish(self.navigatehome)
            self.navigatehome = True

    def usertakeoff_callback(self, empty):
        """Publish alternating bool to bebop user_takeoff."""
        if self.usertakeoff:
            self.pub_usertakeoff.publish(self.usertakeoff)
            self.usertakeoff = False
        else:
            self.pub_usertakeoff.publish(self.usertakeoff)
            self.usertakeoff = True

    def record_start_callback(self, empty):
        """Publish true to bebop record."""
        self.pub_record.publish(True)

    def record_stop_callback(self, empty):
        """Publish false to bebop record."""
        self.pub_record.publish(False)


if __name__ == '__main__':
    rospy.init_node('autoflight_joy', anonymous=True)
    AutoFlightJoy()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
