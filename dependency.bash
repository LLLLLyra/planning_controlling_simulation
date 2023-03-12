source devel/setup.bash

roscore &
sleep 1 &&
(rosrun simulation obstacles_service.py & 
rosrun simulation simulation_chassis_node)