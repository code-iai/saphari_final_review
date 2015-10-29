For the first try: 

run catkin_make in your catkin workspace to biuld the messages
$ catkin_make
$ rosdep update

for the first run of the test example with client run the task list generation first and then the server and client
$ roscd saphari_task_requirements/src
$ python generate_target_list.py
$ python action_server.py
and in a different shell
$ python action_client.py
