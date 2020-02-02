#!/bin/bash
echo "Setting up ROS boilerplate"

cat > /node/setup.cfg << EOM
[develop]
script-dir=\$base/lib/${L2NAME}
[install]
install-scripts=\$base/lib/${L2NAME}
EOM

cd /node && cp /templates/setup.py . && mkdir ${L2NAME} \
  && mv node.py ${L2NAME}/ && touch ${L2NAME}/__init__.py \
  && mkdir resource && touch resource/${L2NAME} \
  && colcon build --symlink-install
