import keyboard

while True:
    if keyboard.is_pressed('w'):
        self.vehicle_local_position.x += 1  # forward
    elif keyboard.is_pressed('s'):
        self.vehicle_local_position.x -= 1  # backward
    elif keyboard.is_pressed('a'):
        self.vehicle_local_position.y -= 1  # left
    elif keyboard.is_pressed('d'):
        self.vehicle_local_position.y += 1  # right
    elif keyboard.is_pressed('q'): # up
        self.vehicle_local_position.z += 1
    elif keyboard.is_pressed('x'): # down
        self.vehicle_local_position.z -= 1