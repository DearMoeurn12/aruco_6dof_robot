import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped, Pose
from pymoveit2.robots import ur as robot
import math
import time
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from std_msgs.msg import Float64MultiArray

class MoveToPose(Node):
    def __init__(self):
        super().__init__('move_to_pose_service')   

        # Create callback group
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names = robot.joint_names(),
            base_link_name = robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name = robot.MOVE_GROUP_ARM,
            callback_group = callback_group,
            execute_via_moveit = True,
        )

        self.moveit2.pipeline_id = "move_group"
        self.moveit2.planner_id = "RRTkConfigDefault"

        self.declare_parameter('moveit2.planning_time', 5.0)
        self.declare_parameter('moveit2.max_velocity', 0.5)
        self.declare_parameter('moveit2.max_acceleration', 0.5)
        self.moveit2.planning_time = 5.0
        # Scale down velocity and acceleration
        self.moveit2.max_velocity = self.get_parameter('moveit2.max_velocity').value
        self.moveit2.max_acceleration = self.get_parameter('moveit2.max_acceleration').value

        self.moveit2.cartesian_avoid_collisions = False
        self.moveit2.cartesian_jump_threshold = 0.0


    def move_to_pose(self , pose , orientation):
        try: 
            assert len(pose) == 3 , "Pose must be a list of 3 elements"
            assert len(orientation) == 4 , "Orientation must be a list of 4 elements"
            self.get_logger().info(f"Received goal pose: {pose} and orientation: {orientation}")
            synchronous = True
            cancel_after_secs = 0.0
    
            self.get_logger().info(f"Received goal pose: {pose} and orientation: {orientation}")
            self.get_logger().info(f"excecute service to move to pose")

            self.moveit2.move_to_pose(
                position= pose,
                quat_xyzw= orientation,
                # cartesian_max_step=0.0025,
                # cartesian_fraction_threshold=0.0,
                tolerance_position=0.001,
                tolerance_orientation=0.001,
            )

            if synchronous:
                # Synchronous execution
                self.moveit2.wait_until_executed()
                self.get_logger().info("Move to pose service executed synchronously") 
            else:
                # Asynchronous execution
                rate = self.create_rate(10)
                while self.moveit2.query_state() != MoveIt2State.EXECUTING:
                    rate.sleep()

                future = self.moveit2.get_execution_future()

                # Cancel execution after a specified time
                if cancel_after_secs > 0.0:
                    self.get_logger().info(f"Cancelling goal after {cancel_after_secs} seconds")
                    self.create_rate(cancel_after_secs).sleep()
                    self.moveit2.cancel_execution()

                # Wait for the future to complete
                while not future.done():
                    rate.sleep()

                # Log the result
                self.get_logger().info(f"Result status: {future.result().status}")
                self.get_logger().info(f"Result error code: {future.result().result.error_code}")
            self.get_logger().info(f"Maniuplation completed successfully")
        except Exception as e:
            self.get_logger().error(f"Error in move_to_pose: {e}")

def main(args=None):   
    rclpy.init(args=args)
    node = MoveToPose()
    # pose xyz = [0.5, 0.5, 0.5] , orientation xyzw = [0.0, 0.0, 0.0, 1.0]
    node.move_to_pose([0.5, 0.5, 0.5], [0.0, 0.0, 0.0, 1.0])
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
