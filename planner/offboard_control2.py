import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus



class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        #self.xpose = 10.0

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        global x2
        global y2
        y2 = y
        x2 = self.vehicle_local_position.x
        min_x = -20
        max_x = 20
        min_y = -20
        max_y = 20
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]

        for z in range(-20,20):
        	if msg.position[0] % 2 != 0 and msg.position[0] < max_x: #odd
        		msg.position[1]= msg.position[1] - 1
        		if msg.position[1] <= min_y:
        			msg.position[1]= min_y
        			msg.position[0] = x2 - 3 
        			msg.position[0] = x2
        			#return
        		#msg.position[1]= msg.position[1] + 1
        		#if msg.position[1] > max_y:
        			#msg.position[1]= max_y
        			#msg.position[0] = x2 + 1
        			#msg.position[0] = x2
        #else: 
        	#elif msg.position[0] % 2 == 0 and msg.position[0] < max_x: #even
        		#msg.position[1]= msg.position[1] + 1
        		#if msg.position[1] > max_y:
        			#msg.position[1]= max_y
        			#msg.position[0] = x2 + 1
        			#msg.position[0] = x2
       
        #n = 1.0
        #while z <= 10.0:
        	#if n == 1.0 and msg.position[0] < max_x: #odd
        		#msg.position[1]= msg.position[1] - 1
        		#if msg.position[1] <= min_y:
        			#msg.position[1]= min_y
        			#if msg.position[1] == min_y:
        				#msg.position[0] = -3
        				#msg.position[0] = msg.position[0]
        				#n = 0.0
        			#continue
        			
        	#elif n == 0.0 and msg.position[0] < max_x: #even
        		#msg.position[1]= msg.position[1] + 1
        		#if msg.position[1] > max_y:
        			#msg.position[1] = max_y
        			#msg.position[0] = msg.position[0] + 3
        			#msg.position[0] = self.vehicle_local_position.x
        			#n = 1.0
        			#continue  
        	#else:
        		#break		
        		   		
        		
        			

        
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

        	self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y, self.takeoff_height)

        if self.offboard_setpoint_counter < 25:
            self.offboard_setpoint_counter += 1
     		
        def timer_callback2(self) -> None:
        	self.vehicle_local_position.x = self.vehicle_local_position.x + 1
	 	

			
def main(args=None) -> None:
    #move()

    #y=y+1
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()
    publish_position_setpoint()
    timer_callback()

    

if __name__ == '__main__':
    try:
        main()
        
    except Exception as e:
        print(e)


        		
        		 
