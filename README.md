
## Ground Plane Extraction
This repo contains code to extract ground planes, calculate their convex hull, and slope. This will be used to map the groud surface as a robot traverses the environment.

## Running the node
All of the code in contained in `src/extract_planes_node.cc` and its accompanying header file `include/extract_planes_node.hh`. There are three launch files included in `launch/` that take care of starting the node. We also utilize [pcl_ros nodelets](http://wiki.ros.org/pcl_ros) to crop the raw input cloud to an area of interest in front of the robot. The nodelets are launched in `launch/pcl_nodelets.launch`. The plane extraction node is launched in `launch/plane_seg.launch`. We create a `main.launch` file which includes both. In practice, we launch the node by running `roslaunch surface_normals main.launch`.
