from rosbags.rosbag1 import Reader, Writer, WriterError, ReaderError
from rosbags.serde import deserialize_cdr, ros1_to_cdr, serialize_cdr, cdr_to_ros1
import matplotlib.pyplot as plt
import numpy as np
import math
import argparse
def main(source_bag: str = './path_test.bag', destination_bag: str = './result.bag', topic: str = '/vslam2d_pose', target_path: int = 100, visualize: bool = False):
    if visualize:
        plt.style.use('seaborn')
        fig_original, ax_original = plt.subplots()
        fig_result, ax_result = plt.subplots()
        ax_original.set_title('Original Path')
        ax_result.set_title('Result Path')
    path_ori = []
    path_result = []
    with Writer(destination_bag) as writer:
        with Reader(source_bag) as reader:
            # messages() accepts connection filters
            connections = [x for x in reader.connections if x.topic == topic]    
            messages = [msg for msg in reader.messages(connections=connections)]    
            step = math.floor(len(messages)/target_path)
            mid_step = math.floor(step/2)
            temp_path = []
            connection, _, _ = messages[0]
            writer_connection = writer.add_connection(topic, connection.msgtype)
            for connection, timestamp, rawdata in messages:
                msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                path_ori.append((msg.pose.position.x, msg.pose.position.y))
                temp_path.append(msg)
                if len(temp_path) == step:
                    new_message = temp_path[mid_step]
                    mean_x = np.mean([msg.pose.position.x for msg in temp_path])
                    mean_y = np.mean([msg.pose.position.y for msg in temp_path])
                    path_result.append((mean_x, mean_y))
                    new_message.pose.position.x = mean_x
                    new_message.pose.position.y = mean_y
                    writer.write(writer_connection, timestamp, cdr_to_ros1(serialize_cdr(new_message, connection.msgtype), connection.msgtype))
                    del temp_path
                    temp_path = []
    if visualize:
        ax_original.plot([pose[0] for pose in path_ori], [pose[1] for pose in path_ori], linewidth=2.0)
        ax_result.plot([pose[0] for pose in path_result], [pose[1] for pose in path_result], linewidth=2.0)
        plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Source bag File")
    parser.add_argument("-d","--destination", help="Destination for new bag file")
    parser.add_argument("-n","--length", help="Number of New Path", default=100, type=int)
    parser.add_argument("-t", "--topic", help="Path Topic", default='/vslam2d_pose')
    parser.add_argument("-v","--visualize", help="Display the source and destionation path using matplotlib", type=bool, default=False)
    args = parser.parse_args()
    source = args.source
    destination = args.destination if args.destination is not None else "".join([f'{source.split(".bag")[0]}_result.bag'])
    length = args.length
    topic = args.topic
    visualize = args.visualize
    try:
        main(source_bag=source, destination_bag=destination, target_path=length, topic=topic, visualize=visualize)
        print(f"{destination} Created Successfully")
    except WriterError as ex:
        print(f"Problem with destination file: {ex}")
    except ReaderError as ex:
        print(f"Problem with source file: {ex}")
    except Exception as ex:
        print(f"Unexpected Problem: {ex}")