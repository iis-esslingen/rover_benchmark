import os
import re
import argparse
from io import BytesIO
from pathlib import Path
from typing import *

import cv2
import numpy
import pandas
import rosbag
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tqdm import tqdm

class SensorDataHandler():
    def _add_images_to_bag(
        self,
        bag,
        input_directory: str,
        topic_name: str,
        encoding: str = "passthrough",
        is_depth: bool = False
    ) -> None:
        
        cv_bridge = CvBridge()
        
        is_grayscale_camera = "t265" in topic_name

        image_filepaths = sorted(
            [
                os.path.join(input_directory, filename)
                for filename in os.listdir(input_directory)
                if os.path.isfile(os.path.join(input_directory, filename)) and filename.endswith('.png')
            ],
            key=lambda image_filepath: self._get_unix_timestamp(image_filepath),
        )

        for index, image_filepath in enumerate(
            tqdm(
                image_filepaths,
                desc=f"Adding topic '{topic_name}'."
            )
        ):
            try:
                if is_grayscale_camera:
                    image = cv2.imread(image_filepath, cv2.IMREAD_GRAYSCALE)
                elif is_depth:
                    image = cv2.imread(image_filepath, cv2.IMREAD_UNCHANGED)
                    if image is not None:
                        image = image.astype(numpy.uint16)
                else:
                    image = cv2.imread(image_filepath, cv2.IMREAD_COLOR)

                if image is None:
                    print(f"Failed to read image: {image_filepath}")
                    continue

                image_message = cv_bridge.cv2_to_imgmsg(image, encoding=encoding)
                image_message_unix_timestamp = self._get_unix_timestamp(image_filepath)
                image_message.header.stamp = rospy.Time.from_sec(image_message_unix_timestamp)
                image_message.header.seq = index + 1
                image_message.header.frame_id = "camera"

                bag.write(topic_name, image_message, image_message.header.stamp)

            except KeyboardInterrupt:
                print("Process interrupted by user. Exiting...")
                break
            except Exception as e:
                print(f"Failed to add data: {image_filepath}. Error: {e}")
                break
        
    def _get_unix_timestamp(self, filepath):
        filename = os.path.basename(filepath)
        timestamp_match = re.match(r"(\d+\.\d+)", filename)
        
        if timestamp_match:
            return float(timestamp_match.group(1))
        else:
            raise ValueError(f"Filename '{filename}' does not contain a valid timestamp.")

    def _add_imu_data_to_bag(
        self,
        imu_data: pandas.DataFrame,
        bag: rosbag.Bag,
        topic_name: str,
    ) -> None:

        imu_data.dropna(inplace=True)

        for index, (timestamp, row) in enumerate(
            tqdm(
                list(imu_data.iterrows()),
                desc=f"Adding topic '{topic_name}'."
            )
        ):
            imu_message = Imu()
            imu_message_unix_timestamp = timestamp.timestamp()
            imu_message.header.stamp = rospy.Time.from_sec(imu_message_unix_timestamp)
            imu_message.header.seq = index + 1
            imu_message.header.frame_id = "imu"
            imu_message.linear_acceleration = Vector3(*list(row.iloc[:3]))
            imu_message.angular_velocity = Vector3(*list(row.iloc[3:]))

            bag.write(topic_name, imu_message, imu_message.header.stamp)


class IntelD435iCameraHandler(SensorDataHandler):

    def _add_data_to_bag(
        self,
        input_directory: str,
        bag: rosbag.Bag,
    ) -> None:

        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "realsense_D435i", "rgb"),
            topic_name="/d435i/rgb_image",
            encoding="bgr8",
        )
        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "realsense_D435i", "depth"),
            topic_name="/d435i/depth_image",
            encoding="16UC1",
            is_depth=True,
        )

        imu_data = pandas.read_csv(
            os.path.join(input_directory, "realsense_D435i", "imu", "imu.txt"), 
            sep=",",
            header=None,
            names=["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
        )

        imu_data = imu_data.set_index(
            pandas.to_datetime(imu_data["timestamp"], unit="s")
        ).drop("timestamp", axis=1)

        self._add_imu_data_to_bag(
            imu_data,
            bag,
            "/d435i/imu",
        )
        
class IntelT265CameraHandler(SensorDataHandler):

    def _add_data_to_bag(
        self,
        input_directory: str,
        bag: rosbag.Bag,
    ) -> None:

        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "realsense_T265", "cam_left"),
            topic_name="/t265/image_left",
            encoding="mono8",
        )
        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "realsense_T265", "cam_right"),
            topic_name="/t265/image_right",
            encoding="mono8",
        )

        imu_data = pandas.read_csv(
            os.path.join(input_directory, "realsense_T265", "imu", "imu.txt"), 
            sep=",",
            header=None,
            names=["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]
        )

        imu_data = imu_data.set_index(
            pandas.to_datetime(imu_data["timestamp"], unit="s")
        ).drop("timestamp", axis=1)

        self._add_imu_data_to_bag(
            imu_data,
            bag,
            "/t265/imu",
        )
        
class PiCameraHandler(SensorDataHandler):
        
    def _add_data_to_bag(
        self,
        input_directory: str,
        bag: rosbag.Bag,
    ) -> None:

        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "pi_cam", "rgb"),
            topic_name="/pi_cam/rgb_image",
            encoding="bgr8",
        )
        
class Vn100ImuHandler(SensorDataHandler):
    
    def _add_data_to_bag(
        self,
        input_directory: str,
        bag: rosbag.Bag,
    ) -> None:
        
        imu_data = pandas.read_csv(
            os.path.join(input_directory, "vn100", "imu.txt"), 
            sep=" ", 
            usecols=range(1, 8),
            names=["timestamp_end_reading", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])
        
        imu_data = imu_data.set_index(
            pandas.to_datetime(imu_data["timestamp_end_reading"], unit="s"),
        ).drop("timestamp_end_reading", axis=1)

        self._add_imu_data_to_bag(
            imu_data, 
            bag, 
            "/vn100/imu"
        )
        
def main():
    parser = argparse.ArgumentParser(description="Script to convert raw sensor data in a rosbag.")
    parser.add_argument("--input_directory", 
                        type=str, 
                        help="Path to the directory containing sensor data.")
    parser.add_argument("--output_bag", 
                        type=str, 
                        help="Path to the output rosbag file. Default set to input_directory/rosbag.bag")
    parser.add_argument("--sensors", 
                        nargs='+', 
                        choices=["d435i", "t265", "pi_cam", "vn100"],
                        default=["d435i", "t265", "pi_cam", "vn100"],
                        help="List of sensors to include in the rosbag. Choices are 'd435i', 't265', 'pi_cam', 'vn100'.")
    args = parser.parse_args()

    output_bag = args.output_bag if args.output_bag else os.path.join(args.input_directory, "rosbag.bag")

    bag = rosbag.Bag(output_bag, 'w')
    
    try:
        sensor_handlers = {
            "d435i": IntelD435iCameraHandler(),
            "t265": IntelT265CameraHandler(),
            "pi_cam": PiCameraHandler(),
            "vn100": Vn100ImuHandler(),
        }

        for sensor in args.sensors:
            handler = sensor_handlers[sensor]
            handler._add_data_to_bag(
                input_directory=args.input_directory,
                bag=bag
            )

    except KeyboardInterrupt:
        print("Process interrupted by user. Exiting...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bag.close()
        print(f"Rosbag saved to: {output_bag}")

if __name__ == "__main__":
    main()
