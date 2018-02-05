# OBJECT DETECTION AND RECOGNITION IN 3D Point Cloud scene


## Nodes
```
	|-> tf_emitter
	|-> obj_detector
	|-> RViz
	`-> pcd_to_pointcloud
```

### tf_emitter
publishes world frame for RViz 

### obj_detector
does  all the object detection and recognition part

### Rviz
used for visualization

### pcd_to_pointcloud
to publish scenes for testing

To run package
```
roslaunch robot_vision robot_vision.launch
```


## Note
```
	/bin - contains .pcd scenes which are broadcasted
```
