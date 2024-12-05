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
        imu_synchronization_strategy: str,
        bag: rosbag.Bag,
        topic_name: str,
    ) -> None:

        accelerometer_frequency = 1 / (
            imu_data.dropna(subset=[imu_data.columns[0]])
            .index.to_series()
            .diff()
            .mean()
            .total_seconds()
        )

        gyroscope_frequency = 1 / (
            imu_data.dropna(subset=[imu_data.columns[1]])
            .index.to_series()
            .diff()
            .mean()
            .total_seconds()
        )

        if imu_synchronization_strategy == "merge":
            columns = imu_data.columns
        elif imu_synchronization_strategy == "downsampling":
            columns = (
                imu_data.columns[:3]
                if accelerometer_frequency > gyroscope_frequency
                else imu_data.columns[3:]
            )
        else:
            columns = (
                imu_data.columns[:3]
                if accelerometer_frequency < gyroscope_frequency
                else imu_data.columns[3:]
            )

        imu_data[columns] = imu_data[columns].apply(
            lambda columns: columns.interpolate(method="time")
        )

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
        imu_synchronization_strategy: str,
    ) -> None:

        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "d435i", "rgb"),
            topic_name="/d435i/rgb_image",
            encoding="bgr8",
        )
        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "d435i", "depth"),
            topic_name="/d435i/depth_image",
            encoding="16UC1",
            is_depth=True,
        )

        accelerometer_filepath = os.path.join(input_directory, "d435i", "accel.csv")
        accelerometer_data = pandas.read_csv(accelerometer_filepath)

        gyroscope_filepath = os.path.join(input_directory, "d435i", "gyro.csv")
        gyroscope_data = pandas.read_csv(gyroscope_filepath)

        imu_data = pandas.merge(
            accelerometer_data,
            gyroscope_data,
            how="outer",
            on="timestamp",
            sort=True,
        )

        imu_data = imu_data.set_index(
            pandas.to_datetime(imu_data["timestamp"], unit="s")
        ).drop("timestamp", axis=1)

        self._add_imu_data_to_bag(
            imu_data,
            imu_synchronization_strategy,
            bag,
            "/d435i/imu",
        )
        
class IntelT265CameraHandler(SensorDataHandler):

    def _add_data_to_bag(
        self,
        input_directory: str,
        bag: rosbag.Bag,
        imu_synchronization_strategy: str,
    ) -> None:

        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "t265", "cam_left"),
            topic_name="/t265/image_left",
            encoding="mono8",
        )
        self._add_images_to_bag(
            bag=bag,
            input_directory=os.path.join(input_directory, "t265", "cam_right"),
            topic_name="/t265/image_right",
            encoding="mono8",
        )

        accelerometer_filepath = os.path.join(input_directory, "t265", "accel.csv")
        accelerometer_data = pandas.read_csv(accelerometer_filepath)

        gyroscope_filepath = os.path.join(input_directory, "t265", "gyro.csv")
        gyroscope_data = pandas.read_csv(gyroscope_filepath)

        imu_data = pandas.merge(
            accelerometer_data,
            gyroscope_data,
            how="outer",
            on="timestamp",
            sort=True,
        )

        imu_data = imu_data.set_index(
            pandas.to_datetime(imu_data["timestamp"], unit="s")
        ).drop("timestamp", axis=1)

        self._add_imu_data_to_bag(
            imu_data,
            imu_synchronization_strategy,
            bag,
            "/t265/imu",
        )
        
class PiCameraHandler(SensorDataHandler):
        
    def _add_data_to_bag(
        self,
        input_directory: str,
        bag: rosbag.Bag,
        imu_synchronization_strategy: str
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
        imu_synchronization_strategy: str
    ) -> None:
        
        imu_data = pandas.read_csv(os.path.join(input_directory, "vn100", "imu.csv"), usecols=range(1, 8))
        
        imu_data = imu_data.set_index(
            pandas.to_datetime(imu_data["timestamp end reading"], unit="s"),
        ).drop("timestamp end reading", axis=1)

        self._add_imu_data_to_bag(
            imu_data, imu_synchronization_strategy, bag, "/vn100/imu"
        )
        
def main():
    parser = argparse.ArgumentParser(description="Script to convert raw sensor data in a rosbag.")
    parser.add_argument("--input_directory", type=str, help="Path to the directory containing sensor data.")
    parser.add_argument("--output_bag", type=str, help="Path to the output rosbag file. Default set to input_directory/rosbag.bag")
    parser.add_argument("--sensors", nargs='+', choices=["d435i", "t265", "pi_cam", "vn100"], required=True,
                        help="List of sensors to include in the rosbag. Choices are 'd435i', 't265', 'pi_cam', 'vn100'.")
    parser.add_argument("--imu_sync_strategy", type=str, choices=["merge", "downsampling", "upsampling"],
                        default="merge", help="IMU synchronization strategy. Default is 'merge'.")
    args = parser.parse_args()

    output_bag = args.output_bag if args.output_bag else os.path.join(args.input_directory, "rosbag.bag")

    bag = rosbag.Bag(args.output_bag, 'w')
    
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
                bag=bag,
                imu_synchronization_strategy=args.imu_sync_strategy
            )

    except KeyboardInterrupt:
        print("Process interrupted by user. Exiting...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        bag.close()
        print(f"Rosbag saved to: {args.output_bag}")

if __name__ == "__main__":
    main()