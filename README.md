# robot_urdf_parser
Parses robot URDF XML


### Input: 
File location of the URDF robot description.


### Functions:
-`show()`: representation of current robot Pandas DF.

-`joint_idx()`: returns Pandas DF of x,y,z locations (in meters) of each joint in initial config.

-`rotation_axis()`: returns axis and direction of rotation for each annotated joint.

-`phys_limits()`: returns length of current link and angle threshholds (radians)

-`node_dist(child,parent)`: return the distance between two named links
