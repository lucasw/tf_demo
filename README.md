# tf_demo

Demostrate ROS tf and BufferCore features

python - tf2_ros only

tf vs. tf_static

tf_static can change, but no time history, no interpolation

# rust

## old_tf_to_new_tf

```
ROS_PACKAGE_PATH=`rospack find geometry_msgs`:`rospack find tf2_msgs`:`rospack find std_msgs`:`rospack find actionlib_msgs` cargo build --release

./target/release/tf2tf _lookup_child:=base_link _broadcast_child:=tmp2
```
