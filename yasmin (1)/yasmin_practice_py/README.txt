
    ros2 launch yasmin_practice_py marsiNav2Turtlebot3House.launch.xml

    ros2 run yasmin_practice_py turtlebotNavYasmin

secuencialmente:
    ros2 service call /marsi_nav_move yasmin_prac_if/srv/Move mode:\ 0\

random:
    ros2 service call /marsi_nav_move yasmin_prac_if/srv/Move mode:\ 1\

en espera:
    ros2 service call /marsi_nav_move yasmin_prac_if/srv/Move mode:\ 10\

cancelar:
    ros2 service call /marsi_nav_cancel_goals std_srvs/srv/Empty {}\ 




