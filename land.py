
import rospy
from std_srvs.srv import Trigger


land = rospy.ServiceProxy('land', Trigger)

land()