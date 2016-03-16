# gazebo_mark_plugin
Gazebo model plugin to move an aruco mark

# Quick setup
Build
```
# Inside a catkin workspace
catkin_make --pkg mark_plugin
```
Download aruco mark model from [here](https://github.com/erlerobot/erle_gazebo_models) and place the `aruco0` folder inside ~/.gazebo/models

Alternatively, add this tag to your mark model (before </model>):
```
<plugin name="mark_driver" filename="libmark_plugin.so">
      <shape>circle</shape>
</plugin>
```

Before adding the model to the world, you can replace `circle` with `square` or `dot`. This will be the shape that will follow the mark once it's inserted.