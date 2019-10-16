# ground_segmentation

Implementation of the ground segmentation algorithm proposed in 
```
@inproceedings{himmelsbach2010fast,
  title={Fast segmentation of 3d point clouds for ground vehicles},
  author={Himmelsbach, Michael and Hundelshausen, Felix V and Wuensche, H-J},
  booktitle={Intelligent Vehicles Symposium (IV), 2010 IEEE},
  pages={560--565},
  year={2010},
  organization={IEEE}
}
```
The `ground_segmentation` package contains the ground segmentation library with a ROS interface to use it.

The library is compiled separately from the ROS interface if you're not using ROS.

## Installation

Requires the following dependencies to be installed:

- *ROS* (Melodic)
- *PCL*

Compile using your favorite catkin build tool (e.g. `catkin build ground_segmentation`)

## Launch instructions

The ground segmentation ROS node can be launch by executing `roslaunch ground_segmentation segmentation.launch`.
Input and output topic names can be specified in the same file.

## Parameter description

Parameters are set in `ground_segmentation/launch/segmentation_params.yaml`

### Ground Condition

- **max_dist_to_line**  maximum vertical distance of point to line to be considered ground.
- **max_slope**  Maximum slope of a line.
- **max_fit_error**  Maximum error a point is allowed to have in a line fit.
- **max_start_height**  Maximum height difference between new point and estimated ground height to start a new line.
- **long_threshold**  Distance after which the max_height condition is applied.
- **max_height**  Maximum height difference between line points when they are farther apart than *long_threshold*.
- **sensor_height**  Sensor height above ground.
- **line_search_angle**  How far to search in angular direction to find a line. A higher angle helps fill "holes" in the ground segmentation.

### Segmentation

- **r_min**  Distance at which segmentation starts.
- **r_max**  Distance at which segmentation ends.
- **n_bins**  Number of radial bins.
- **n_segments**  Number of angular segments.

### Other

- **n_threads**  Number of threads to use.
- **latch**  Latch output point clouds in ROS node. 
- **visualize** Visualize the segmentation result. **ONLY FOR DEBUGGING.** Do not set true during online operation.

## TODO

- [ ] use mean point instead of lowest-Z ?
- [ ] run segmentation from a different coordinates than (0, 0, 0)
- [ ] Use non-linear range binning
