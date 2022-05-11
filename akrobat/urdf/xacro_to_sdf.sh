#!/bin/bash
rm -rf akrobat.sdf akrobat.urdf
rosrun xacro xacro akrobat.xacro > akrobat.urdf
gz sdf -p akrobat.urdf > akrobat.sdf