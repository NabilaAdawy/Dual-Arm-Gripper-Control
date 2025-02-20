#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ur_msgs.srv import SetIO
from functools import partial
from ur_msgs.msg import IOStates
import time

class ClutchSub(Node):
    def __init__(self):
        super().__init__("Bob_Subscriber")
        self.clutch_subscriber = self.create_subscription(
            Bool, "/footswitch3", self.clutch_sub_callback, 10) 
        
        self.io_state_subsriber = self.create_subscription(
            IOStates, "/bob/io_and_status_controller/io_states", self.state_sub_callback, 10) 
        
        self.gripper_closed = None
        self.prev_clutch_state = False
        self.power_gripper()
    
    def power_gripper(self):
        self.call_setio_service(1, 0, 1.0)
    
    def clutch_sub_callback (self, msg: Bool): 

        if self.gripper_closed is None:
            return

        clutch_data = msg.data
        
        if clutch_data and not self.prev_clutch_state:

            if self.gripper_closed: 
                self.get_logger().info("Open Bob's Gripper")
                self.call_setio_service(1, 2, 0.0)
                time.sleep(0.015)
                self.call_setio_service(1, 1, 1.0)
                time.sleep(0.015)

                self.gripper_closed = False
            else:
                self.get_logger().info("Close Bob's Gripper")
                self.call_setio_service(1, 1, 0.0)
                time.sleep(0.015)
                self.call_setio_service(1, 2, 1.0)
                time.sleep(0.015)

                self.gripper_closed = True

        self.prev_clutch_state = clutch_data
                

    def state_sub_callback(self, msg: IOStates):
        if len(msg.digital_out_states) > 2:
            gripper_state = msg.digital_out_states[2].state
            if self.gripper_closed is None:
                self.gripper_closed = gripper_state
                self.get_logger().info(f"Bob's Initial gripper state set to: {'Closed' if self.gripper_closed else 'Open'}")


        
    def call_setio_service(self, fun, pin, state): 
        client = self.create_client(SetIO, "/bob/io_and_status_controller/set_io")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for the service .. ")
        
        request = SetIO.Request() 
        request.fun = fun 
        request.pin = pin 
        request.state = state
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_setio)) # calls callback_setio when the service has replied
        
    def callback_setio(self, future): 
        try: 
            response = future.result()
        except Exception as e:
            self.get_logger().error("service call failed: %r" % (e,))
        

def main(args = None):
    rclpy.init(args=args)
    node = ClutchSub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
