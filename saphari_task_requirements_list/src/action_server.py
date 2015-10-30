#! /usr/bin/env python

import roslib; roslib.load_manifest('saphari_task_requirements_list')
import rospy

import actionlib

import saphari_task_requirements_list
from saphari_task_requirements_list.msg import SlotGoal, SlotAction, SlotActionFeedback, SlotActionResult,\
    PerceptionAction, PerceptionActionFeedback, PerceptionActionResult, ItemStatus, Item, TaskList, TaskState

import json
from rospy_message_converter import message_converter


class TaskListActionServer(object):
    # create messages that are used to publish feedback/result
    _slot_feedback = SlotActionFeedback()
    _slot_result = SlotActionResult()
    _perception_feedback = PerceptionActionFeedback()
    _perception_result = PerceptionActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as_perception = actionlib.SimpleActionServer(self._action_name+"_perception_action",
                                                           PerceptionAction,
                                                           execute_cb=self.run_perception_action,
                                                           auto_start=False)
        self._as_slot = actionlib.SimpleActionServer(self._action_name+"_slot_action",
                                                     SlotAction,
                                                     execute_cb=self.run_slot_action,
                                                     auto_start=False)
        self._as_perception.start()
        self._as_slot.start()
        self.task_list_publisher = rospy.Publisher("TaskList", TaskList, queue_size=10)
        # self._task_list_msg = TaskList()
        self._task_list_msg = self.load_target_packing_list("task_list.json")
        self.task_list_publisher.publish(self._task_list_msg)

        self.task_state_publisher = rospy.Publisher("TaskState", TaskState, queue_size=10)
        self._task_state_list_msg = TaskState()
        # for target_item in self._task_list_msg.target_items:
        #     item = Item()
        #     item.info = target_item.info
        #     item.state.status.status = ItemStatus.UNKNOWN
        #     item.target = target_item
        #     self._task_item_state_list_msg.items.append(item)
        self.task_state_publisher.publish(self._task_state_list_msg)
        rospy.loginfo("TaskPackingListServer is running")

    def load_target_packing_list(self, file_path):
        print "load target list: %s" % "/".join(__file__.split("/")[:-1]) + "/" + file_path
        f = open("/".join(saphari_task_requirements_list.__file__.split("/")[:-1]) + "/res/" + file_path, 'r+')
        json_str = f.read()
        msg_dict = json.loads(json_str)

        return message_converter.convert_dictionary_to_ros_message('saphari_task_requirements_list/TaskList', msg_dict)

    def store_target_packing_list(self, file_path):
        pass

    def load_task_state_list(self, file_path):
        pass

    def store_task_state_list(self, file_path):
        pass

    def get_item_with_item_id(self, item_id):
        item = None
        for item_state in self._task_state_list_msg.items:
            # print item_state.info
            if item_state.info.id == item_id:
                item = item_state
                break
        return item

    def get_free_target(self, kind):

        item_of_kind_needed = 0
        _target_item = None
        for item in self._task_list_msg.target_items:
            if item.kind == kind:
                item_of_kind_needed += 1
                _target_item = item
        item_of_kind_in = 0
        for item in self._task_state_list_msg.items:
            if item.kind == kind and item.state.status.status == ItemStatus.IN_SLOT:
                item_of_kind_in += 1
        if item_of_kind_in < item_of_kind_needed and _target_item:
            return _target_item
        else:
            return None

    def run_perception_action(self, goal):
        # helper variables
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, on items %s' % (self._action_name+"_perception_action", goal.perceived))
        self._perception_feedback.status = 1
        # remove all item which are not in the final slot
        to_remove = []
        for item in self._task_state_list_msg.items:  # per reference
            if item.state.status.status == ItemStatus.DETECTED:
                to_remove.append(item)
        for elem in to_remove:
            self._task_state_list_msg.items.remove(elem)

        # add actual perception state
        for equipment in goal.perceived:
            new_item = Item()
            new_item.kind = equipment.ID
            new_item.state.pose = equipment.pose
            new_item.state.status.status = ItemStatus.DETECTED
            # find free target slot
            free_target = self.get_free_target(new_item.kind)
            if free_target:
                print "free_traget is left over"
                new_item.target = free_target
            else:
                print "NO free_traget is left over"
            # append to task_state_list
            self._task_state_list_msg.items.append(new_item)

        self._as_perception.publish_feedback(self._perception_feedback)
        self.task_state_publisher.publish(self._task_state_list_msg)

        if success:
            self._perception_result.result.state = self._task_state_list_msg
            rospy.loginfo('%s: Succeeded' % self._action_name+"_perception_action")
            self._as_perception.set_succeeded(self._perception_result.result)

    def run_slot_action(self, goal):
        # helper variables
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, action %i on item with id %s' % (self._action_name+"_slot_action",
                                                                       goal.action_type,
                                                                       goal.item))
        # start executing the action
        self._slot_feedback.status = 1
        item = Item()
        if goal.action_type == SlotGoal.ITEM_INSERT:
            item.kind = goal.item.ID
            item.state.pose = goal.item.pose
            item.state.slot_id = goal.slot_id
            item.state.status.status = ItemStatus.IN_SLOT
            free_target = self.get_free_target(item.kind)
            if free_target:
                print "free_traget is left over"
                item.target = free_target
            else:
                print "NO free_traget is left over"
            self._task_state_list_msg.items.append(item)
        elif goal.action_type == SlotGoal.ITEM_REMOVED:
            item = self.get_item_with_item_id(goal.item_state.info.id)
            item_to_remove = None
            for list_item in self._task_state_list_msg.items:
                if list_item.status.status == ItemStatus.IN_SLOT and \
                        list_item.state.slot_id == goal.slot_id and list_item.state.kind == goal.item.ID:
                    item_to_remove = list_item
            if item_to_remove is None:
                rospy.logwarn("It was tried to remove a item from Target Tray while no item of kind {0} and in slot {1} can be found".format(goal.item.ID, goal.slot_id))
            else:
                self._task_state_list_msg.items.remove(item_to_remove)
        else:
            rospy.logwarn("no action defined")
            # self._as_slot.set_rejected()
            self._slot_feedback.status = -1

        rospy.loginfo("new status of item with name: {0} is: {1}".format(item.target.info.name, item.state.status.status))

        self._as_slot.publish_feedback(self._slot_feedback)
        self.task_state_publisher.publish(self._task_state_list_msg)

        if success and not self._slot_feedback.status == -1:
            self._slot_result.result.state = self._task_state_list_msg
            rospy.loginfo('%s: Succeeded' % self._action_name+"_slot_action")
            self._as_slot.set_succeeded(self._slot_result.result)
        else:
            self._slot_result.result.state = self._task_state_list_msg
            rospy.logwarn('%s: Aborted' % self._action_name+"_slot_action")
            self._as_slot.set_aborted(self._slot_result.result)


if __name__ == '__main__':
    rospy.init_node('task_requirements_list')
    TaskListActionServer(rospy.get_name())
    rospy.spin()
