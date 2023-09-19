import time
import numpy as np
from motor import *


if __name__ == '__main__':
    init_connection()
    enable_torque()

    # Tests
    t0 = time.time()
    while time.time() - t0 < 6:
        set_position(1, np.pi/2 * np.sin(np.pi * time.time() - t0), write_only=False)
    print("Done!")

    # Freeing the motors
    set_position(2, 0, write_only=False)
    time.sleep(1)

    disable_torque()
    close_connection()
