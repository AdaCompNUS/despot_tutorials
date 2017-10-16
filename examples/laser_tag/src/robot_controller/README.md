# Robot Controller

A high-level interface to send actions to the robot, and receive obseravtions.

#### Test Controllers
Run a test to see if you can control the robots:
```bash
$ rosrun laser_tag test_laser_tag_controller
```
The first robot (bottom left) should move North. And the other robot should try and run away. After executing the action, you should see a print-out of the laser-readings:

```
[ INFO] [1507790509.778345756, 1131.223000000]: Laser Observations
[ INFO] [1507790509.778420017, 1131.223000000]: North: 0
[ INFO] [1507790509.778456571, 1131.223000000]: East: 7
[ INFO] [1507790509.778491221, 1131.223000000]: South: 2
[ INFO] [1507790509.778523052, 1131.223000000]: West: 3
[ INFO] [1507790509.778550735, 1131.223000000]: NorthEast: 4
[ INFO] [1507790509.778578519, 1131.223000000]: SouthEast: 4
[ INFO] [1507790509.778604918, 1131.223000000]: SouthWest: 4
[ INFO] [1507790509.778631267, 1131.223000000]: NorthWest: 5
```

See [test_laser_tag_controller.cpp](examples/laser_tag/test/test_laser_tag_controller.cpp) for details on giving an action to the robot, and receiving observations.

#### Command-Line Interface
You can also send actions to the robot directly using rosservice
```bash
$ rosservice call /laser_tag_action_obs "action: 1"
```
