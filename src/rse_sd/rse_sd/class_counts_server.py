import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray
from rse_sd_msgs.msg import ClassCount

from rse_sd_msgs.srv import GetClassCount

import time

class GetClassCountsServer(Node):

    def __init__(self):
        super().__init__('get_class_counts_server')
        self.srv = self.create_service(GetClassCount, 'get_class_count', self.get_class_count_callback)

        # Create detections subscriber
        self.subscription = self.create_subscription(
            Detection2DArray,
            'detections',
            self.detections_callback,
            10)
        
        self.detections_dict = {}
        
    def detections_callback(self, msg):
        for detection in msg.detections:
            class_name = detection.results[0].hypothesis.class_id
            if class_name in self.detections_dict:
                self.detections_dict[class_name] += 1
            else:
                self.detections_dict[class_name] = 1
        self.get_logger().info(f"Updated detections dictionary: {self.detections_dict}")

    def get_class_count_callback(self, request, response):
        if request.class_name == 'all':
            for class_name, count in self.detections_dict.items():
                response.counts.append(self._create_class_count(class_name, count))
        else:
            count = self.detections_dict.get(request.class_name)
            if count is not None:
                response.counts.append(self._create_class_count(request.class_name, count))
            else:
                self.get_logger().warn(f"Class '{request.class_name}' not found.")
                response.counts = []
        
        self .get_logger().info(f"Returning class counts for class {request.class_name}: {response.counts}")
        
        # Sleep for 5 seconds to simulate a long-running service
        for i in range(5):
            time.sleep(1)
            self.get_logger().info(f"Sleeping for {i+1} seconds...")
        
        return response
        
    def _create_class_count(self, class_name, count):
        class_count = ClassCount()
        class_count.class_name = class_name
        class_count.count = count
        return class_count
    


def main():
    rclpy.init()

    class_counts_server = GetClassCountsServer()

    rclpy.spin(class_counts_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()