<h1 align="center">
  The ROVER Visual SLAM Benchmark

  
  
  [Project Page](https://iis-esslingen.github.io/rover/) | [Paper](https://arxiv.org/pdf/2412.02506)
</h1>

<br>

## Prerequisites
The only required software is Docker. Each SLAM method comes with its own Docker container, making setup straightforward. We recommend using **VSCode** with the Docker extension for an enhanced development experience.

## SLAM Methods
Each method is available as a Docker container.

### DPVO & DPV-SLAM
**Note:** The container currently does not support visualization.

To run the evaluation:

```bash
python evaluate_rover \
    --base_data_path /garden_small/2023-08-18 \
    --ground_truth_path /garden_small/2023-08-18/ground_truth.txt \
    --output_path ./rover_trajectories \
    --cameras d435i t265 pi_cam \
    --trials 5
```

To enable Loop Closing for DPV-SLAM, add the argument: ```--opts LOOP_CLOSURE True```.

### DROID-SLAM

Separate scripts are provided for each camera in the ```DROID-SLAM/evaluation_scripts``` folder.

Example:
```bash
python evaluation_scripts/test_rover_d435i.py \
    --data_path /garden_small/2023-08-18 \
    --ground_truth_path /garden_small/2023-08-18/ground_truth.txt \
    --output_path ./rover_trajectories
```

To test DROID-SLAM in RGBD mode (Camera D435i), add the flag ```--depth```, for Stereo mode (Camera T265) add ```--stereo```.

### OpenVINS

To launch the application:

```bash
roslaunch ov_msckf <launch_file> \
    do_bag:=<do_bag> bag:=<bag> \
    do_save_traj:=<do_save_traj> \
    traj_file_name:=<traj_file_name>
```

#### Parameters:

- `launch_file`: Specifies the launch file to use. Choices include:
    - `"rover_mono-inertial_d435i_external.launch"`
    - `"rover_mono-inertial_d435i_internal.launch"`
    - `"rover_mono-inertial_pi-cam-02_external.launch"`
    - `"rover_mono-inertial_t265_external.launch"`
    - `"rover_mono-inertial_t265_internal.launch"`
    - `"rover_stereo-inertial_t265_external.launch"`
    - `"rover_stereo-inertial_t265_internal.launch"`

- `do_bag`: *(Optional)* Specifies whether to replay a bag. Set to either:
    - `"true"`: To replay a bag.
    - `"false"`: To not replay a bag.

- `bag`: *(Optional)* Specifies the path to the rosbag file.

- `do_save_traj`: *(Optional)* Specifies whether to save a predicted trajectory. Set to either:
    - `"true"`: To save the trajectory.
    - `"false"`: To not save the trajectory.

- `traj_file_name`: *(Optional)* Specifies the file path where the estimated trajectory should be saved.


### VINS-Fusion

To launch the application:

```bash
roslaunch vins <launch_file> \
    do_bag:=<do_bag> bag:=<bag> \
    do_save_traj:=<do_save_traj> \
    traj_file_name:=<traj_file_name> \
    do_lc:=<enable_loop_closing>
```

#### Parameters:

- `launch_file`: Specifies the launch file to use. Choices include:
    - `"rover_mono-inertial_d435i_external.launch"`
    - `"rover_mono-inertial_d435i_internal.launch"`
    - `"rover_mono-inertial_pi-cam-02_external.launch"`
    - `"rover_mono-inertial_t265_external.launch"`
    - `"rover_mono-inertial_t265_internal.launch"`
    - `"rover_stereo_t265.launch"`
    - `"rover_stereo-inertial_t265_external.launch"`
    - `"rover_stereo-inertial_t265_internal.launch"`

- `do_bag`: *(Optional)* Specifies whether to replay a bag. Set to either:
    - `"true"`: To replay a bag.
    - `"false"`: To not replay a bag.

- `bag`: *(Optional)* Specifies the path to the rosbag file.

- `do_save_traj`: *(Optional)* Specifies whether to save a predicted trajectory. Set to either:
    - `"true"`: To save the trajectory.
    - `"false"`: To not save the trajectory.

- `traj_file_name`: *(Optional)* Specifies the file path where the estimated trajectory should be saved.

- `do_lc`: *(Optional)* Specifies whether to enable loop closing. Set to either:
    - `"true"`: To enable loop closing.
    - `"false"`: To disable loop closing.


### SVO-Pro

To launch the application:

```bash
roslaunch svo_ros <launch_file> \
    do_bag:=<do_bag> bag:=<bag> \
    do_save_traj:=<do_save_traj> \
    traj_file_name:=<traj_file_name> \
    do_lc:=<enable_loop_closing>
```

#### Parameters:

- `launch_file`: Specifies the launch file to use. Choices include:
    - `"rover_mono_d435i.launch"`
    - `"rover_mono_pi-cam-02.launch"`
    - `"rover_mono_t265.launch"`
    - `"rover_mono-inertial_d435i_external.launch"`
    - `"rover_mono-inertial_d435i_internal.launch"`
    - `"rover_mono-inertial_pi-cam-02_external.launch"`
    - `"rover_mono-inertial_t265_external.launch"`
    - `"rover_mono-inertial_t265_internal.launch"`
    - `"rover_stereo_t265.launch"`
    - `"rover_stereo-inertial_t265_external.launch"`
    - `"rover_stereo-inertial_t265_internal.launch"`

- `do_bag`: *(Optional)* Specifies whether to replay a bag. Set to either:
    - `"true"`: To replay a bag.
    - `"false"`: To not replay a bag.

- `bag`: *(Optional)* Specifies the path to the rosbag file.

- `do_save_traj`: *(Optional)* Specifies whether to save a predicted trajectory. Set to either:
    - `"true"`: To save the trajectory.
    - `"false"`: To not save the trajectory.

- `traj_file_name`: *(Optional)* Specifies the file path where the estimated trajectory should be saved.

- `do_lc`: *(Optional)* Specifies whether to enable loop closing. Set to either:
    - `"true"`: To enable loop closing.
    - `"false"`: To disable loop closing.


### ORB-SLAM3

To launch the application:

```bash
roslaunch orb_slam3_ros <launch_file> \
    do_bag:=<do_bag> bag:=<bag> \
    do_save_traj:=<do_save_traj> \
    traj_file_name:=<traj_file_name> \
    do_lc:=<enable_loop_closing>
```

#### Parameters:

- `launch_file`: Specifies the launch file to use. Choices include:
    - `"rover_mono_d435i.launch"`
    - `"rover_mono_pi-cam-02.launch"`
    - `"rover_mono_t265.launch"`
    - `"rover_mono-inertial_d435i_external.launch"`
    - `"rover_mono-inertial_d435i_internal.launch"`
    - `"rover_mono-inertial_pi-cam-02_external.launch"`
    - `"rover_mono-inertial_t265_external.launch"`
    - `"rover_mono-inertial_t265_internal.launch"`
    - `"rover_stereo_t265.launch"`
    - `"rover_stereo-inertial_t265_external.launch"`
    - `"rover_stereo-inertial_t265_internal.launch"`

- `do_bag`: *(Optional)* Specifies whether to replay a bag. Set to either:
    - `"true"`: To replay a bag.
    - `"false"`: To not replay a bag.

- `bag`: *(Optional)* Specifies the path to the rosbag file.

- `do_save_traj`: *(Optional)* Specifies whether to save a predicted trajectory. Set to either:
    - `"true"`: To save the trajectory.
    - `"false"`: To not save the trajectory.

- `traj_file_name`: *(Optional)* Specifies the file path where the estimated trajectory should be saved.

- `do_lc`: *(Optional)* Specifies whether to enable loop closing. Set to either:
    - `"true"`: To enable loop closing.
    - `"false"`: To disable loop closing.

## Utils

### raw_to_rosbag.py

`raw_to_rosbag.py` is a Python script designed to convert raw sensor data into a ROS bag file. This tool is useful for working with robotics datasets, enabling streamlined integration with ROS-based tools and workflows.

The script supports various sensors and offers customization options through command-line arguments.

### Command Syntax

```bash
python raw_to_rosbag.py \
    --input_directory <input_directory> \
    --output_bag <output_bag> \
    --sensors <sensor_list> \
    [--imu_sync_strategy <imu_sync_strategy>]
```

| **Argument**            | **Type**      | **Required** | **Description**                                                                                                          |
|-------------------------|---------------|--------------|--------------------------------------------------------------------------------------------------------------------------|
| `input_directory`       | `str`         | Yes          | Path to the directory containing raw sensor data.                                                                         |
| `output_bag`            | `str`         | No           | Path to the output ROS bag file. Defaults to `<input_directory>/rosbag.bag`.                                              |
| `sensors`               | `list[str]`   | Yes          | List of sensors to include in the ROS bag. Choices are: `d435i`, `t265`, `pi_cam`, and `vn100`.                           |
| `imu_sync_strategy`     | `str`         | No           | IMU synchronization strategy. Choices are: `merge` (default), `downsampling`, or `upsampling`.                            |

The `--imu_sync_strategy` parameter defines how to synchronize IMU data from multiple sensors. The available options are:

- **merge (default)**: This strategy combines IMU data from multiple sources by fusing the accelerometer and gyrometer readings. It ensures that the data is aligned and integrated into a single stream.
  
- **downsampling**: This strategy reduces the frequency of IMU data to match the lowest rate among the available sensors. It can be useful when the sensors operate at different frequencies, and you want to ensure synchronization at a lower rate.

- **upsampling**: This strategy increases the frequency of IMU data to match the highest rate among the available sensors. It interpolates data to achieve a higher frequency, ensuring synchronization at the rate of the fastest sensor.

## Citing
If you find our work useful, please consider citing:
```bibtex
@article{schmidt2024rover,
      title={ROVER: A Multi-Season Dataset for Visual SLAM}, 
      author={Fabian Schmidt and Constantin Blessing and Markus Enzweiler and Abhinav Valada},
      year={2024},
      eprint={2412.02506},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2412.02506}, 
}
```
