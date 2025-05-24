#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from planner_msgs.srv import pci_initialization

REQUIRED_TOPICS = [
    "/quad_0/odom",
    "/quad0_pcl_render_node/cloud"
]

def wait_for_required_topics(required_topics, timeout=10.0):
    rospy.loginfo("Checking for required topics...")
    start_time = rospy.Time.now()
    rate = rospy.Rate(2)  # check at 2Hz
    while not rospy.is_shutdown():
        published = dict(rospy.get_published_topics())
        missing = [topic for topic in required_topics if topic not in published]

        if not missing:
            return True

        if (rospy.Time.now() - start_time).to_sec() > timeout:
            return False

        rate.sleep()

def main():
    rospy.init_node('auto_service_trigger')

    if not wait_for_required_topics(REQUIRED_TOPICS, timeout=10.0):
        rospy.logerr("Essential topics missing. Aborting service calls.")
        return

    rospy.wait_for_service('/pci_initialization_trigger')
    rospy.wait_for_service('/planner_control_interface/std_srvs/automatic_planning')

    try:
        init_srv = rospy.ServiceProxy('/pci_initialization_trigger', pci_initialization)
        rospy.loginfo("Calling pci_initialization_trigger...")
        init_resp = init_srv()
        rospy.sleep(2.0)

        plan_srv = rospy.ServiceProxy('/planner_control_interface/std_srvs/automatic_planning', Trigger)
        rospy.loginfo("Calling automatic_planning...")
        plan_resp = plan_srv()
        rospy.loginfo("Planning response: %s", plan_resp.message)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    main()
