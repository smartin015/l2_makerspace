OLDIFS=$IFS; IFS=','
for i in l2base,. l2base-pre,./pre l2base-post,./post l2base-post-sim,./post-sim; do
  set -- $i
  echo "\n\n\ndocker build --tag $1 $2"
  docker build --tag $1 $2
  if [ $? -ne 0 ]; then
    echo "==== BUILD FAILED for $1 ===="
    exit 1
  fi
done
IFS=$OLDIFS
