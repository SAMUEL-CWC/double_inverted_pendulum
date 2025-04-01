import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rse_sd_msgs.action import FindObject


class FindObjectActionClient(Node):

    def __init__(self):
        super().__init__('find_object_action_client')
        self._action_client = ActionClient(self, FindObject, 'find_object')

    def send_goal(self, video_path, max_frames, class_name, confi_threshold=0.9):
        goal_msg = FindObject.Goal()
        goal_msg.video_path = video_path
        goal_msg.max_frames = max_frames
        goal_msg.class_name = class_name
        goal_msg.confi_threshold = confi_threshold
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return None


        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._action_client.send_goal_async(goal_msg)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().error('Goal failed')
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.frames_number))


def main(args=None):
    rclpy.init(args=args)

    action_client = FindObjectActionClient()

    # Get user input for video path, max frames, class name, and confidence threshold
    video_path = input("Enter the path to the video file: ")
    max_frames = int(input("Enter the maximum number of frames to process: "))
    class_name = input("Enter the class name to search for: ")
    confi_threshold = float(input("Enter the confidence threshold (0-1): "))
    
    # Send the goal to the action server
    future = action_client.send_goal(video_path, max_frames, class_name, confi_threshold)
    if future is None:
        action_client.get_logger().error('Failed to send goal')
        return
    # Wait for the result
    # rclpy.spin_until_future_complete(action_client, future)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()