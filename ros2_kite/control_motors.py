# This file should support everything to do with both sending commands
# and receiving sensor information back from the sensors
# key object is that it can be operated and tested independently of the vision and kite detection
# setup

# proposed to move quite a few parts here from basic_motion_detection including
# 1) setup of the base and controls
# 2) standalone operation
# 3) normal operation will be to accept and return values to the vision setup

# Example file showing a basic pygame "game loop"
import pygame

def standalone(command = 'N'):
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
        pygame.draw.rect(screen, "blue", (200,150,100,50))

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            player_pos.y -= 300 * dt
        if keys[pygame.K_s]:
            player_pos.y += 300 * dt
        if keys[pygame.K_a]:
            player_pos.x -= 300 * dt
        if keys[pygame.K_d]:
            player_pos.x += 300 * dt

        # fill the screen with a color to wipe away anything from last frame

        # RENDER YOUR GAME HERE

        # flip() the display to put your work on screen
        pygame.display.flip()
        dt = clock.tick(60) / 1000

    return()


if __name__ == "__main__":
    standalone(command = 'N')
    pygame.quit()