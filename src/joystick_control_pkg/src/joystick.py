import pygame
import time

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()  # Initialize the joystick module

    # Check for joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joysticks detected")
        return None
    else:
        # Initialize the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"Joystick found: {joystick.get_name()}")
        return joystick

def read_joystick_axes(joystick):
    pygame.event.pump()  # Process event queue

    # Define dead zone threshold
    dead_zone_threshold = 0.05

    # Read each axis
    for i in range(joystick.get_numaxes()):
        axis_value = joystick.get_axis(i)
        
        # Implement dead zone
        if -dead_zone_threshold < axis_value < dead_zone_threshold:
            axis_value = 0

        print(f"Axis {i}: {round(axis_value, 4)}")

if __name__ == "__main__":
    joystick = initialize_joystick()

    if joystick:
        try:
            while True:
                read_joystick_axes(joystick)
                time.sleep(0.1)  # Adjust the sleep time as needed
        except KeyboardInterrupt:
            print("Program exited")
