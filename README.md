# AGV Master

ROS package that communicates with cytron_jetracer package.

agv_node.py includes OccupancyGrid map, A* algorithm for path generation, pure pursuit controller, subscribers for Marvelmind indoor GPS systems, and publishers for communication with Jetracer.

Run the following command to run the node:

```
rosrun agv_master agv_node.py
```
