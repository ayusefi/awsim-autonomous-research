#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from multi_object_tracker_msgs.msg import TrackedObjectArray
from visualization_msgs.msg import MarkerArray

class TrackerValidator(Node):
    def __init__(self):
        super().__init__('tracker_validator')
        
        # Counters
        self.tracked_objects_count = 0
        self.markers_count = 0
        self.last_tracked_objects_count = 0
        self.last_markers_count = 0
        
        # Subscribers
        self.tracked_objects_sub = self.create_subscription(
            TrackedObjectArray,
            '/tracked_objects',
            self.tracked_objects_callback,
            10
        )
        
        self.markers_sub = self.create_subscription(
            MarkerArray,
            '/tracked_objects_markers',
            self.markers_callback,
            10
        )
        
        # Timer to periodically check and report
        self.timer = self.create_timer(2.0, self.check_sync)
        
        self.get_logger().info("Tracker Validator Node started")
        self.get_logger().info("Monitoring /tracked_objects and /tracked_objects_markers topics")
    
    def tracked_objects_callback(self, msg):
        self.tracked_objects_count = len(msg.objects)
        self.get_logger().info(f"Received TrackedObjectArray with {self.tracked_objects_count} objects")
    
    def markers_callback(self, msg):
        # Count non-DELETE markers
        active_markers = [m for m in msg.markers if m.action != m.DELETE]
        self.markers_count = len(active_markers)
        self.get_logger().info(f"Received MarkerArray with {self.markers_count} active markers (total: {len(msg.markers)})")
    
    def check_sync(self):
        if self.tracked_objects_count != self.markers_count:
            self.get_logger().warn(
                f"MISMATCH: TrackedObjects: {self.tracked_objects_count}, "
                f"Markers: {self.markers_count}"
            )
        else:
            self.get_logger().info(
                f"SYNC OK: Both have {self.tracked_objects_count} objects/markers"
            )
        
        # Update last counts
        self.last_tracked_objects_count = self.tracked_objects_count
        self.last_markers_count = self.markers_count

def main(args=None):
    rclpy.init(args=args)
    
    validator = TrackerValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
