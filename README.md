# DAT160 Search & Rescue – group 8

Semester project in DAT160: a team of Turtlebot3 robots in Gazebo has to find five ArUco markers (fires, a human and a big fire) in a maze within 10 minutes, report their positions and meet at the big fire.

The robots use the blueprint map to work out where they need to stand for the camera to see every wall, split those points between them and drive there with A*. A leader node (`mission`) decides what each robot does, and stops everything once all five markers are reported and two robots have been at the big fire.

## Requirements

ROS2 Foxy on Ubuntu 20.04, with Gazebo and Turtlebot3.

These course packages must be in the same workspace (`~/ros2_ws/src`):

| Package | What we use from it |
| --- | --- |
| `multi_robot_challenge_23` | The blueprint maps (`maps/`) and the robot model (`urdf/`) |
| `ros2_aruco` and `ros2_aruco_interfaces` | `aruco_node` and the `ArucoMarkers` message type |
| `multi_robot_scoring` (`scoring` and `scoring_interfaces`) | The `SetMarkerPosition` service type, and the scoring node when started with `scoring:=true` |

`scoring_interfaces` must always be built, since our nodes use the service type. The scoring node itself is only needed if you start it yourself.

You also need `nav2_map_server`, `nav2_lifecycle_manager`, `cv_bridge`, `python3-opencv` and `python3-numpy`.

## Setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/h597236/dat160-SaR-Gr8.git
cd ~/ros2_ws
colcon build --packages-select scoring_interfaces scoring search_and_rescue_comp_26
source install/setup.bash
```

The world files are in our package (`worlds/`). They include the `libgazebo_ros_state.so` plugin, which scoring needs to see where the robots are.

## Running

Terminal 1, start the simulation (replace `x` with map 1–5):

```bash
ros2 launch search_and_rescue_comp_26 sim.launch.py world:=x
```

Terminal 2, start the robot team:

```bash
ros2 launch search_and_rescue_comp_26 team.launch.py speed:=0.4
```

The scoring node is off by default, since it is usually started separately (for example by the teacher). To start it together with the team for testing:

```bash
ros2 launch search_and_rescue_comp_26 team.launch.py speed:=0.4 scoring:=true
```

The robots wait until `/set_marker_position` is available before they move, so without a scoring node they stand still.

| Argument | Default | What |
| --- | --- | --- |
| `world` (sim) | `1` | Map 1–5 |
| `robots` (sim and team) | `4` | Number of robots, 1–5, same number in both |
| `speed` (team) | `0.26` | Forward speed (m/s) |
| `scoring` (team) | `false` | Start the scoring node together with the team |
| `use_wall_check` (team) | `true` | Skip non-white walls faster while scanning |
| `mission` (team) | `true` | `false` = only navigator and reporter, for sending goals by hand |

## Nodes

| File | What it does |
| --- | --- |
| `navigator.py` | One per robot. Plans a path with A* on the map and follows it. The only node that publishes `cmd_vel`. |
| `coverage.py` | Not a node. Finds the points that together see every wall (greedy set cover). |
| `mission.py` | The leader. One state machine per robot: `explore`, `to_vp`, `scan`, `inspect`, `approach`, `go_big_fire`, `wait_big_fire`, `done`. |
| `marker_reporter.py` | Works out where the markers are in the map and reports each ID once to scoring. |
| `wall_check.py` | Uses HSV to check whether the wall straight ahead is white or not. |

### marker_reporter

- Reads `/<robot>/aruco_markers` directly and looks up TF **at the time the image was taken**. The course node `marker_recognition` uses where the robot is *now*, which gives the wrong position when the robot is moving.
- Only uses detections within 3.0 m and 65° viewing angle. The limits come from our tests of the ArUco detection.
- Averages 5 detections and reports with z = 0, since scoring compares in 3D against z = 0.
- If the same ID is rejected 3 times, it is treated as already reported (for example after the node has been restarted).

| Topic | Type | Direction | Content |
| --- | --- | --- | --- |
| `/<robot>/aruco_markers` | `ros2_aruco_interfaces/ArucoMarkers` | in | Markers in the camera image |
| `/set_marker_position` | `scoring_interfaces/SetMarkerPosition` | service, out | Report to scoring |
| `/reported_markers` | `std_msgs/Int8MultiArray` | out, every second | IDs accepted by scoring |
| `/big_fire_position` | `geometry_msgs/Point` | out, every second | Position of ID 4 once accepted |

## RViz

`sim.launch.py` opens RViz with `rviz/team.rviz`. It shows each robot's path and the coverage points (blue = remaining, grey = done).
