# Aprendiendo como simular con pybullet integrando ros2 humble 

#En una terminal ejecutar

cd my_ws

colcon build

source install/setup.bash

ros2 run pybullet_sim pybullet_node

#En otra terminal diferente ejecutar 
cd my_ws

colcon build

source install/setup.bash

ros2 run pybullet_sim controller_node
