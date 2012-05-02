screen -d -m -r -S mysession
screen -X -S mysession -p 1 stuff "~/code/robotics/cwru-ros-pkg/cwru_semi_stable; autotest"
screen -X -S mysession -p 1 stuff "roslaunch cwru_bringup.launch; autotest"
screen -X -S mysession -p 2 stuff "~/code/robotics/cwru-ros-pkg/cwru_semi_stable/cwru_nav; autotest"
screen -X -S mysession -p 2 stuff "roslaunch start_amcl_2ndfloor.launch; autotest"

 








