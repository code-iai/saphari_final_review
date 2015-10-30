#! /usr/bin/env python

#import roslib; roslib.load_manifest('actionlib_tutorials')
import os
import roslib; roslib.load_manifest('saphari_task_requirements_list')
import rospy

# messages to construct a list of item targets and store this list as goal for the task.
import saphari_task_requirements_list.msg
import saphari_task_requirements_list as task_packing_list

import json
from rospy_message_converter import message_converter
if not os.path.exists("/".join(task_packing_list.__file__.split("/")[:-1]) + "/res"):
    os.mkdir("/".join(task_packing_list.__file__.split("/")[:-1]) + "/res")
file_path = "/".join(task_packing_list.__file__.split("/")[:-1]) + "/res/task_list.json"

WITH_PUBLISHING = False
task_list_publisher = None
if WITH_PUBLISHING:
    rospy.init_node('task_packing_list')
    task_list_publisher = rospy.Publisher("TaskList", task_packing_list.msg.TaskList, queue_size=10)
_task_list_msg = task_packing_list.msg.TaskList()
print "TaskPackingList gets generated"

new_target_item = task_packing_list.msg.ItemTarget()
new_target_item.info.name = 'Schere'
new_target_item.info.description = 'schneidet'
new_target_item.kind = task_packing_list.msg.Item.SCISSORS
new_target_item.slot_id = 1
#new_target_item.info.id = 1002
# new_target_item.pose.pos[:] = [-0.5, -0.1, 0.1]
_task_list_msg.target_items.append(new_target_item)

new_target_item = task_packing_list.msg.ItemTarget()
new_target_item.info.name = 'Bowl'
new_target_item.info.description = 'kann man falten'
new_target_item.kind = task_packing_list.msg.Item.BOWL
new_target_item.slot_id = 2
#new_target_item.info.id = 1001
# new_target_item.pose.pos[:] = [-0.5, -0.2, 0.1]
_task_list_msg.target_items.append(new_target_item)

new_target_item = task_packing_list.msg.ItemTarget()
new_target_item.info.name = 'Hammer'
new_target_item.kind = task_packing_list.msg.Item.HAMMER
new_target_item.slot_id = 3
#new_target_item.info.id = 1000
new_target_item.info.description = 'If you get hit it hurts otherwise it is fun.'
# new_target_item.pose.pos[:] = [-0.5, -0.3, 0.1]
_task_list_msg.target_items.append(new_target_item)

if WITH_PUBLISHING:
    task_list_publisher.publish(_task_list_msg)

msg_dict = message_converter.convert_ros_message_to_dictionary(new_target_item)
print msg_dict

msg_dict = message_converter.convert_ros_message_to_dictionary(_task_list_msg)
json_str = json.dumps(msg_dict, sort_keys=True, indent=2, separators=(',', ': '))
f = open(file_path, 'w+')
f.write(json_str)
f.close()

print "\n#################\nFinished -> file was written %s\n\n" % file_path

print 60*"#"
print json_str

f = open(file_path, 'r+')
json_str = f.read()
msg_dict = json.loads(json_str)

message_converter.convert_dictionary_to_ros_message('saphari_task_requirements_list/TaskList', msg_dict)
print "#### %s" % message_converter.convert_dictionary_to_ros_message('saphari_task_requirements_list/TaskList', msg_dict)
print "####\n %s" % message_converter.convert_dictionary_to_ros_message('saphari_task_requirements_list/ItemTarget', msg_dict["target_items"][0])
print "Test Finished -> file can be read and TaskList is generated as message %s" % file_path
