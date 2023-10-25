#!/usr/bin/env python


import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def move_to_pose_callback(data):
    try:
        global move_group, gripper_frame
    
    	gripper_pose_stamped = PoseStamped()
   	    gripper_pose_stamped.header.frame_id = 'base_link'
  	    gripper_pose_stamped.header.stamp = rospy.Time.now()
    	gripper_pose_stamped.pose = data
    	rospy.loginfo("Info Recieved!")

    	move_group.moveToPose(gripper_pose_stamped, gripper_frame)
   	    result = move_group.get_move_action().get_result()

    	if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Move Success!")
            else:
                rospy.logerr("Arm goal in state: %s", move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")
    except Exception as e:
        rospy.logerr("Error in callback: {}".format(e))


if __name__ == '__main__':
    rospy.init_node("moveit_bridge")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    # Define ground plane
    # This creates objects in the planning scene that mimic the ground
    # If these were not in place gripper could hit the ground
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    # This is the wrist link not the gripper itself
    gripper_frame = 'wrist_roll_link'

    # Create a "pose_stamped" message as required by moveToPose
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    

    # Subscribe to custom topic to receive the pose messages from matlab
    rospy.Subscriber("/end_effector_pose", Pose, move_to_pose_callback)
    # keep running and listen
    rospy.spin()

    move_group.get_move_action().cancel_all_goals()


