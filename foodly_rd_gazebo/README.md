# foodly_rd_gazebo

[Gazebo](https://gazebosim.org/home) simulation package for Foodly Type R.

## Start-up Node

Run the following command will start Gazebo, which will display the Foodly Type R model, Table, and Box.

When you start up for the first time, the Table and Box models are downloaded, so it may take some time to display the models.

There is no need to connect to a real robot or run `foodly_rd_examples/launch/demo.launch.py`.

```sh
ros2 launch foodly_rd_gazebo foodly_rd_with_table.launch.py
```

```sh
# When fixing base link to world link
ros2 launch foodly_rd_gazebo foodly_rd_with_table.launch.py fix_base_link:=true
```
