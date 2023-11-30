       
	

	self.vehicle_local_position.x = 0.0
        self.vehicle_local_position.y = 0.0
        self.vehicle_local_position.z = -5.0
       

    def manual_control(self, key):
        try:
            if key.char == 'w':
                self.publish_position_setpoint(self.vehicle_local_position.x + 1, self.vehicle_local_position.y, self.vehicle_local_position.z)
            elif key.char == 's':
                self.publish_position_setpoint(self.vehicle_local_position.x - 1, self.vehicle_local_position.y, self.vehicle_local_position.z))
            elif key.char == 'a':
                self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y - 1, self.vehicle_local_position.z))
            elif key.char == 'd':
                self.publish_position_setpoint(self.vehicle_local_position.x, self.vehicle_local_position.y + 1, self.vehicle_local_position.z))
            elif key.char == 'q':
                self.publish_position_setpoint(self.vehicle_local_position.x + 1, self.vehicle_local_position.y, self.vehicle_local_position.z + 1)
            elif key.char == 'x':
                self.publish_position_setpoint(self.vehicle_local_position.x + 1, self.vehicle_local_position.y, self.vehicle_local_position.z - 1,)
        except AttributeError:
            pass
