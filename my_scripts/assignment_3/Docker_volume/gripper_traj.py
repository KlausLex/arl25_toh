import rosbag
import matplotlib.pyplot as plt
import numpy as np
import argparse

def plot_gripper_trajectory(bag_file, topic='/gravity_compensation_controller/traj_joint_states'):
    """
    Reads a ROS bag file, extracts the gripper trajectory from the specified topic,
    and plots the trajectory.

    Args:
        bag_file (str): Path to the ROS bag file.
        topic (str): ROS topic containing the joint states (default: '/gravity_compensation_controller/traj_joint_states').
                     It is assumed that the last element of the joint states is the gripper position.
    """
    gripper_positions = []
    time = []

    try:
        bag = rosbag.Bag(bag_file, 'r')
        print(f"Reading data from bag file: {bag_file}")
        
        for topic, msg, t in bag.read_messages(topics=[topic]):
            gripper_pos = msg.position[-1]  # Assuming gripper position is the last element
            gripper_positions.append(gripper_pos)
            time.append(msg.header.stamp.to_sec())
        
        bag.close()

        if not gripper_positions:
            print("No gripper data found in the bag file.")
            return

        # Convert lists to numpy arrays
        gripper_positions = np.array(gripper_positions)
        print(f"Gripper position over 0.1:{np.where(-gripper_positions > 0.001)[0][0]}")
        time = np.array(time)
        time = time - time[0] # start time at 0

        # Plotting
        plt.figure(figsize=(10, 6))
        plt.plot(time, gripper_positions)
        plt.xlabel('Time (s)')
        plt.ylabel('Gripper Position')
        plt.title('Gripper Trajectory')
        plt.grid(True)
        plt.show()

    except rosbag.ROSBagException as e:
        print(f"Could not open or read bag file: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot gripper trajectory from a ROS bag file.')
    parser.add_argument('bag_file', help='Path to the ROS bag file')
    parser.add_argument('--topic', help='ROS topic containing joint states', default='/gravity_compensation_controller/traj_joint_states')
    args = parser.parse_args()

    plot_gripper_trajectory(args.bag_file, args.topic)