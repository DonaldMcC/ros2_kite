# This file should support everything to do with both sending commands
# and receiving sensor information back from the sensors
# key object is that it can be operated and tested independently of the vision and kite detection
# setup

# proposed to move quite a few parts here from basic_motion_detection including
# 1) setup of the base and controls
# 2) standalone operation
# 3) normal operation will be to accept and return values to the vision setup

def standalone(command = 'N'):
    while command != 'X':
        pass
    return()


if __name__ == "__main__":
    standalone()
