import sys

from rse_sd_msgs.srv import GetClassCount

import rclpy
from rclpy.node import Node


class GetClassCoutnsClient(Node):

    def __init__(self):
        super().__init__('get_class_counts_client')
        self.cli = self.create_client(GetClassCount, 'get_class_count')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetClassCount.Request()

    def send_request(self, class_name):
        self.req.class_name = class_name
        return self.cli.call_async(self.req) # future 


def main():
    rclpy.init()

    get_class_count_client = GetClassCoutnsClient()
    # Chech if there are enough arguments
    if len(sys.argv) != 2:
        get_class_count_client.get_logger().info('usage: class_counts_client class_name')
        class_name = "all"
    else:
        class_name = sys.argv[1]
        get_class_count_client.get_logger().info(f'Requesting class count for {class_name}')
    future = get_class_count_client.send_request(class_name)
    rclpy.spin_until_future_complete(get_class_count_client, future)
    response = future.result()
    get_class_count_client.get_logger().info(
        'Class count for class %s: %s' % (class_name, response.counts))

    get_class_count_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()