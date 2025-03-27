#!/usr/bin/env python3
import yaml

from joint_state_recorder_panda import JointStatesRecorderPanda
import os
import argparse
import rclpy


if __name__ == "__main__":
    argument_parser = argparse.ArgumentParser(description="Run the parameter optimizer")
    argument_parser.add_argument(
        "--config-file",
        "-c",
        help="config file with the ip, admin and password of the robots",
    )
    argument_parser.add_argument(
        "--save_folder_name",
        "-o",
        help="file name which you want to save",
    )

    args = argument_parser.parse_args()
    config_file = args.config_file
    save_folder_name = args.save_folder_name

    absolute_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_folder = os.path.join(absolute_path, save_folder_name)
    if data_folder and not os.path.exists(data_folder):
        os.makedirs(data_folder)

    config_path = os.path.join(absolute_path, "config", config_file)
    config = yaml.load(open(config_path, "r"), Loader=yaml.FullLoader)

    rclpy.init()
    node = JointStatesRecorderPanda(data_folder, config)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except:
        pass
        # traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()
