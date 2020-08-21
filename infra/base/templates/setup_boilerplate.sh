#!/bin/bash
echo "Setting up ROS boilerplate: $@"

# Point ros/colcon to the location of the
# built module
cat > setup.cfg << EOM
[develop]
script-dir=\$base/lib/${L2PKG}
[install]
install-scripts=\$base/lib/${L2PKG}
EOM

# Move all requested files into the python module
mkdir -p ${L2PKG}
for path in $(echo $@); do 
  mv $(pwd)/$path ${L2PKG}/
done

# Move all other *py files into the python module
for path in $(ls $(pwd)/*.py); do
  mv $path ${L2PKG}/
done

# Bunch of required files for ros/colcon to correctly
# find the python module
cp /templates/setup.py .
sed -i "s/\$NODES/$(echo $@)/" setup.py 

# Init file necessary to indicate this is a python module
touch ${L2PKG}/__init__.py
mkdir -p resource && touch resource/${L2PKG} \

colcon build
