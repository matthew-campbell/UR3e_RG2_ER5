#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('gripper_object_attach')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach and /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Detach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
    detach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")


    rospy.loginfo("Attaching object to gripper")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "rg2_eef_link"
    req.model_name_2 = "object"
    req.link_name_2 = "object"
    attach_srv.call(req)


    rospy.loginfo("Detaching object from gripper")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "rg2_eef_link"
    req.model_name_2 = "object"
    req.link_name_2 = "object"
    attach_srv.call(req)
