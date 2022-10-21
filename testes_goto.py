
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

navigate_wait(z=1, auto_arm=True, frame_id='body')

while not rospy.is_shutdown():
    try:
        position = input("Posicao [x, y, z]: ")

        x, y, z = position.split(',')

        navigate_wait(int(x), int(y), int(z), frame_id='map')
    except KeyboardInterrupt:
        print("\nLANDING")
        land()
        break
