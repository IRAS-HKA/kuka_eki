import rclpy
from rclpy.node import Node
from iras_srvs.srv import Pose as PoseSrv
from geometry_msgs.msg import Pose as PoseMsg
from std_srvs.srv import Trigger
import copy


class RobotClient(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.move_cli = self.create_client(PoseSrv, '/move_to_pose')
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move service not available, waiting again...')
        self.opem_cli = self.create_client(Trigger, '/open_gripper')
        while not self.opem_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('opem service not available, waiting again...')
        self.close_cli = self.create_client(Trigger, '/close_gripper')
        while not self.close_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('close service not available, waiting again...')

    def send_move_request(self, pose, cart=False):
        print(pose)
        req = PoseSrv.Request()
        req.pose = pose
        req.cart = cart
        future = self.move_cli.call_async(req)
        return future

    def send_open_request(self):
        req = Trigger.Request()
        future = self.opem_cli.call_async(req)
        return future

    def send_close_request(self):
        req = Trigger.Request()
        future = self.close_cli.call_async(req)
        return future


def wait_for_response(future, minimal_client):
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if future.done():
            try:
                response = future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
                return None
            else:
                return response


def main(args=None):
    rclpy.init(args=args)

    vacuum_gripper_pose = PoseMsg()
    vacuum_gripper_pose.position.x = 0.043
    vacuum_gripper_pose.position.y = 0.281
    vacuum_gripper_pose.position.z = 1.129
    vacuum_gripper_pose.orientation.x = 0.60911007
    vacuum_gripper_pose.orientation.y = 0.6091002
    vacuum_gripper_pose.orientation.z = 0.353106
    vacuum_gripper_pose.orientation.w = -0.36510001

    minimal_client = RobotClient()

    pre_pose = copy.deepcopy(vacuum_gripper_pose)
    pre_pose.position.y = vacuum_gripper_pose.position.y - 0.02
    future_1 = minimal_client.send_move_request(pre_pose)
    response = wait_for_response(future_1, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    future_2 = minimal_client.send_move_request(vacuum_gripper_pose, True)
    response = wait_for_response(future_2, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    future = minimal_client.send_close_request()
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))
    #
    post_pose = copy.deepcopy(vacuum_gripper_pose)
    post_pose.position.z = vacuum_gripper_pose.position.z + 0.02
    future = minimal_client.send_move_request(post_pose, True)
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    out_pose = copy.deepcopy(post_pose)
    out_pose.position.y = post_pose.position.y - 0.04
    future = minimal_client.send_move_request(out_pose, True)
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    future = minimal_client.send_move_request(post_pose, True)
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    future = minimal_client.send_move_request(vacuum_gripper_pose, True)
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))
    #
    future = minimal_client.send_open_request()
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    future = minimal_client.send_move_request(pre_pose, True)
    response = wait_for_response(future, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    home_pose = PoseMsg()
    home_pose.position.x = 0.05
    home_pose.position.y = -0.052
    home_pose.position.z = 1.385
    home_pose.orientation.x = 0.0
    home_pose.orientation.y = 1.0
    home_pose.orientation.z = 0.0
    home_pose.orientation.w = 0.0
    future_3 = minimal_client.send_move_request(home_pose)
    response = wait_for_response(future_3, minimal_client)
    minimal_client.get_logger().info('Result: ' + str(response.success))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
