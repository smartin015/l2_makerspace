#!/bin/bash
cp main.gd rvl.py
echo "Replace all func with def..."
sed -i 's/^func /def /g' rvl.py

echo "Remove all occurrences of var..."
sed -i 's/var //g' rvl.py

echo "Replace all PYTHON comments with their body and remove the next line..."
sed -i 's/#PYTHON: //g' rvl.py

echo "Remove all lines with #PYRM"
sed -i 's/^.*#PYRM.*//g' rvl.py

echo "Done"
