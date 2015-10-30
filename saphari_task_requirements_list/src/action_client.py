#! /usr/bin/env python

import roslib; roslib.load_manifest('saphari_task_requirements_list')
import rospy

# Brings in the SimpleActionClient
import actionlib

# goal message and the result message.
import saphari_task_requirements_list
from saphari_task_requirements_list.msg import PerceptionGoal, SlotGoal, Item

import json
from rospy_message_converter import message_converter


class TaskListPerceptionActionClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('task_requirements_list_perception_action', saphari_task_requirements_list.msg.PerceptionAction)

        self.client.wait_for_server()
        self.actual_goal = PerceptionGoal()

    def send_goal(self):
        self.client.send_goal(self.actual_goal)

    def new_action(self):
        self.actual_goal = PerceptionGoal()

    def wait_for_result(self):
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

    def get_result(self):
        # Prints out the result of executing the action
        self.client.wait_for_result()
        return self.client.get_result()


class TaskListSlotActionClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('task_requirements_list_slot_action', saphari_task_requirements_list.msg.SlotAction)

        self.client.wait_for_server()
        self.actual_goal = SlotGoal()

    def send_goal(self):
        self.client.send_goal(self.actual_goal)

    def new_action(self):
        self.actual_goal = SlotGoal()

    def wait_for_result(self):
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

    def get_result(self):
        # Prints out the result of executing the action
        self.client.wait_for_result()
        return self.client.get_result()
        

def load_target_packing_list(file_path):
    print "load target list: %s" % "/".join(__file__.split("/")[:-1]) + "/" + file_path
    f = open("/".join(saphari_task_requirements_list.__file__.split("/")[:-1]) + "/res/" + file_path, 'r+')
    json_str = f.read()
    msg_dict = json.loads(json_str)

    return message_converter.convert_dictionary_to_ros_message('saphari_task_requirements_list/TaskList', msg_dict)


def run_task_action_client_test():
    task_perception_ac = TaskListPerceptionActionClient()
    task_list = load_target_packing_list("task_list.json")
    print task_list
    from saphari_msgs.msg import Equipment
    perceived_equipment = Equipment()
    perceived_equipment.ID = task_list.target_items[0].kind
    task_perception_ac.actual_goal.perceived.append(perceived_equipment)
    task_perception_ac.send_goal()
    task_perception_ac.wait_for_result()
    print task_perception_ac.get_result()
    print "\n\n "+50*"#" + " \n\n"

    ####################################################################
    import time
    time.sleep(1)
    task_perception_ac.new_action()
    perceived_equipment = Equipment()
    perceived_equipment.ID = task_list.target_items[0].kind
    task_perception_ac.actual_goal.perceived.append(perceived_equipment)

    perceived_equipment = Equipment()
    perceived_equipment.ID = task_list.target_items[0].kind
    task_perception_ac.actual_goal.perceived.append(perceived_equipment)

    task_perception_ac.send_goal()
    task_perception_ac.wait_for_result()
    print task_perception_ac.get_result()
    print "\n\n "+50*"#" + " \n\n"

    ####################################################################
    import time
    time.sleep(1)
    task_slot_ac = TaskListSlotActionClient()
    task_slot_ac.new_action()
    task_slot_ac.actual_goal.action_type = task_slot_ac.actual_goal.ITEM_INSERT
    task_slot_ac.actual_goal.item.ID = task_list.target_items[0].kind
    task_slot_ac.actual_goal.slot_id = task_list.target_items[0].slot_id

    task_slot_ac.send_goal()
    task_slot_ac.wait_for_result()
    print task_slot_ac.get_result()
    print "\n\n "+50*"#" + " \n\n"

    ####################################################################
    time.sleep(1)
    task_perception_ac.new_action()
    perceived_equipment = Equipment()
    perceived_equipment.ID = task_list.target_items[0].kind
    task_perception_ac.actual_goal.perceived.append(perceived_equipment)

    perceived_equipment = Equipment()
    perceived_equipment.ID = task_list.target_items[0].kind
    task_perception_ac.actual_goal.perceived.append(perceived_equipment)

    perceived_equipment = Equipment()
    perceived_equipment.ID = task_list.target_items[1].kind
    task_perception_ac.actual_goal.perceived.append(perceived_equipment)

    task_perception_ac.send_goal()
    task_perception_ac.wait_for_result()
    print task_perception_ac.get_result()

    ####################################################################
    time.sleep(1)
    task_perception_ac.new_action()
    task_perception_ac.send_goal()
    task_perception_ac.wait_for_result()
    print task_perception_ac.get_result()

if __name__ == '__main__':
    Fibonacci = False
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('run_saphari_task_requirements_list_client_py')
        rospy.loginfo('run task action client test')
        run_task_action_client_test()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
