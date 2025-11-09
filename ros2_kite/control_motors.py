# This file should support everything to do with both sending commands
# and receiving sensor information back from the sensors
# key object is that it can be operated and tested independently of the vision and kite detection
# setup

# proposed to move quite a few parts here from basic_motion_detection including
# 1) setup of the base
# 2) standalone operation with a separate control loop based on pygame
# 3) normal operation will be to accept and return values to the vision setup

import pygame
from base_class import Base
from ComArduino2PY3 import init_arduino
import PID  # https://github.com/ivmech/ivPID
from kite_funcs import get_action

base = Base(kitebarratio=1, safety=True)

# for now set to False when no arduino - but may want a full mock setup soon
serial_conn = init_arduino("COM7", 57600, False)

# pid setup from basic_motion_detection
pid = PID.PID(1, 0, 0)
pid.setSampleTime(0.01)


# setup for call from basic_motion_detection - still just updating the base with values so no need to return stuff
def kiteloop(control):
    global base
    doaction = True if control.motortest or base.calibrate or control.inputmode == 3 else False

    if not doaction:
        # TODO Look at what this doaction stuff is all about
        pid.SetPoint = base.targetbarangle
        pid.update(base.barangle)
        base.action = get_action(pid.output, base.barangle)

    # this should get the current angle and then send the message to the arduino
    # below and the PID stuff can all move to control_motors me thinks and then just
    # call a function there to do this all
    if serial_conn:
        base.update_barangle(serial_conn)
    return


def standalone():
    global base
    # pygame setup
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    running = True
    dt = 0
    player_pos = pygame.Vector2(screen.get_width() / 2, screen.get_height() / 2)

    while running:
        # poll for events
        # pygame.QUIT event means the user clicked X to close your window
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill("purple")
        pygame.draw.circle(screen, "red", player_pos, 40)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            player_pos.y -= 300 * dt
        if keys[pygame.K_s]:
            player_pos.y += 300 * dt
        if keys[pygame.K_a]:
            player_pos.x -= 300 * dt
        if keys[pygame.K_d]:
            player_pos.x += 300 * dt

        # flip() the display to put your work on screen
        pygame.display.flip()
        dt = clock.tick(60) / 1000
    return()


if __name__ == "__main__":
    standalone()
    pygame.quit()