# L2 storage node

## Test DB setup

```
docker exec -it storage_db_1 psql app_db app_user -f /volume/test_db.sql
```

```
ros2 service call /l2/storage/project l2_msgs/L2Project '{method: "POST", project: {id: 4, name: "Test proj"}}'

ros2 service call /l2/storage/project l2_msgs/L2Project '{method: "GET", project: {name: "Test proj"}}'
``
