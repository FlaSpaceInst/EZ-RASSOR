# rosrun ez_rassor_control ez_controller.py 
roslaunch ez_rassor_gazebo ez_rassor_world.launch

sleep 5

rosrun joy joy_node &
rosrun ez_rassor_control arms.py &
rosrun ez_rassor_control wheels.py &
rosrun ez_rassor_control drums.py &
rosrun ez_rassor_control ez_controller.py &