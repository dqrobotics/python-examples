#!/bin/python3
"""
(C) Copyright 2023 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)

Instructions:
    Prerequisites:
	    - dqrobotics
	    - dqrobotics-interface-vrep

1) Open the CoppeliaSim scene synch_test.ttt
2) Run and enjoy!
"""

from dqrobotics.interfaces.vrep import DQ_VrepInterface
import time

vi = DQ_VrepInterface()


def main() -> None:
    try:
        vi.connect("127.0.0.1", 19997, 100, 10)
        vi.set_synchronous(True)
        vi.start_simulation()
        time.sleep(0.1)
        time_simulation_step = 0.05
        y_0 = vi.get_object_pose("/Sphere").translation().vec3()[2]
        print("---------------------------")
        print("Initial height: ", y_0)
        print("---------------------------")
        for i in range(0, 6):
            t = i * time_simulation_step
            y_sim = vi.get_object_pose("/Sphere").translation().vec3()[2]
            y_est = y_0 - 0.5 * 9.81 * pow(t, 2)
            vi.trigger_next_simulation_step()
            vi.wait_for_simulation_step_to_end()
        print("Elapsed time: ", t)
        print("Estimated height: ", y_est, "Measured height: ", y_sim)
        vi.stop_simulation()
        vi.disconnect()

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

        vi.stop_simulation()
        vi.disconnect()


if __name__ == "__main__":
    main()
