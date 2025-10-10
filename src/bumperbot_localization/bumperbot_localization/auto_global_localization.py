#!/usr/bin/env python3
# auto_global_localization.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import time

CANDIDATE_SERVICES = [
    '/reinitialize_global_localization',  # used by nav2 BT plugin example
    '/reinitialize_global_localization_srv',
    '/global_localization',
    '/amcl/reinitialize_global_localization',
    '/amcl/global_localization'
]

class AutoGlobalLocalization(Node):
    def __init__(self):
        super().__init__('auto_global_localization')
        self.get_logger().info('AutoGlobalLocalization starting, will search for AMCL global-localize service...')
        # wait a bit for nodes to come up
        time.sleep(2.0)
        available = self.get_node_names_and_namespaces()  # quick wake-up to rclpy
        # try to find and call one of the candidate services
        svc_list = self.get_service_names_and_types()
        svc_names = [n for n,_ in svc_list]
        for s in CANDIDATE_SERVICES:
            if s in svc_names:
                self.get_logger().info(f'Found service {s} -- calling it to trigger global localization')
                client = self.create_client(Empty, s)
                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().warn(f'Service {s} not available (timeout)')
                    continue
                req = Empty.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                if future.done():
                    self.get_logger().info(f'Called {s} successfully')
                else:
                    self.get_logger().warn(f'Call to {s} did not complete')
                break
        else:
            self.get_logger().warn('No global-localization service found. Check AMCL/service names with `ros2 service list`.')
        # done: shutdown
        self.get_logger().info('AutoGlobalLocalization finished; node exiting.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AutoGlobalLocalization()
    # node will call service then exit in its constructor
    # ensure proper shutdown
    try:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
