
import rospy
from clover import srv

navigate = rospy.ServiceProxy('navigate', srv.Navigate)

navigate(z=1, auto_arm=True, frame_id='body')
