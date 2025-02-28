import os
import rclpy
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory

class BagUnpack(Node):

    def __init__(self):
        super().__init__('bag_unpack')
        config_file_path = os.path.join(
            get_package_share_directory('bag_unpack'),
            'config',
            'config.yaml'
        )
        
        self.declare_parameter('input_bag_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('included_topics', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('output_bag_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('excluded_tf', rclpy.Parameter.Type.STRING_ARRAY)

        self.bag_path = self.get_parameter('input_bag_path').get_parameter_value().string_value
        self.included_topics = self.get_parameter('included_topics').get_parameter_value().string_array_value
        self.output_bag_path = self.get_parameter('output_bag_path').get_parameter_value().string_value
        self.excluded_transformations = self.get_parameter('excluded_tf').get_parameter_value().string_array_value
        self.excluded_transformations = set(tuple(tf_pair.split(':2:')) for tf_pair in self.excluded_transformations if ':2:' in tf_pair)

        self._exit = False

        if not os.path.isdir(self.bag_path):
            self.get_logger().error(f"Input directory does not exist: {self.bag_path}")
            self._exit = True
            return
        
        if os.path.isdir(self.output_bag_path):
            self.get_logger().error(f"Output directory already exists: {self.output_bag_path}")
            self._exit = True
            return

        db3_files = [file for file in os.listdir(self.bag_path) if file.endswith('.db3')]
        yaml_files = [file for file in os.listdir(self.bag_path) if file.endswith('.yaml')]

        if len(db3_files) != 1 or len(yaml_files) != 1:
            self.get_logger().error(f"Input directory contains invalid number or db3/yaml files. Make sure it contains one file of that types.")
            self._exit = True
            return

    def is_ok(self):
        return not self._exit

    def filter_bag(self):
        storage_id = "sqlite3"
        storage_options, converter_options = self.get_rosbag_options(self.bag_path, storage_id)
        storage_options_out, converter_options_out = self.get_rosbag_options(self.output_bag_path, storage_id)

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        writer = rosbag2_py.SequentialWriter()
        writer.open(storage_options_out, converter_options_out)

        for topic in reader.get_all_topics_and_types():
            if topic.name in self.included_topics:
                writer.create_topic(topic)

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == '/tf' or topic == '/tf_static' and topic in self.included_topics:
                msg = deserialize_message(data, TFMessage)
                filtered_transforms = [
                    tf for tf in msg.transforms 
                    if (tf.header.frame_id, tf.child_frame_id) not in self.excluded_transformations
                ]
                if filtered_transforms:
                    msg.transforms = filtered_transforms
                    writer.write(topic, serialize_message(msg), t)
            elif topic in self.included_topics:
                writer.write(topic, data, t)
        self.get_logger().info('Filtering complete.')

    @staticmethod
    def get_rosbag_options(path, storage_id, serialization_format='cdr'):
        storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format)
        return storage_options, converter_options


def main():
    rclpy.init()
    sbr = BagUnpack()
    if sbr.is_ok():
        sbr.filter_bag()
    sbr.destroy_node()
    rclpy.shutdown()
