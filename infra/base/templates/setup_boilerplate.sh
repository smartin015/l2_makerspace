#!/bin/bash
echo "Setting up ROS boilerplate"

cd node

# Point ros/colcon to the location of the
# built module
cat > setup.cfg << EOM
[develop]
script-dir=\$base/lib/${L2NAME}
[install]
install-scripts=\$base/lib/${L2NAME}
EOM

# Move all requested files into the python module
mkdir -p ${L2NAME}
for path in $(echo $@); do 
  mv $path ${L2NAME}/
done
ls ${L2NAME}/

# Bunch of required files for ros/colcon to correctly
# find the python module
cp /templates/setup.py .
touch ${L2NAME}/__init__.py
mkdir -p resource && touch resource/${L2NAME} \
  
colcon build --symlink-install
