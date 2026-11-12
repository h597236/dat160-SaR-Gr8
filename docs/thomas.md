# Thomas' solution

The robots use the blueprint map to work out where they need to stand for the camera to see every wall. They split those points between them and drive there with A*. A leader node controls what each robot does, and stops everything once all five markers are found and the big fire meeting is done.

## Nodes

| File | What it does |
| --- | --- |
| `navigator.py` | One per robot. Plans a path with A* on the map and follows it with go-to-point from Bug2 (`fix_yaw`, `go_straight`). The lidar picks up obstacles that are not in the map, such as other robots. The only node that publishes `cmd_vel`. |
| `coverage.py` | Not a node. Finds the wall faces that must be seen, what each candidate point can see (1.0–2.3 m, under 55°, clear line of sight), and picks points greedily until 95 % is covered (set cover). |
| `mission.py` | The leader. One state machine per robot: `explore`, `to_vp`, `scan`, `inspect`, `approach`, `go_big_fire`, `wait_big_fire`, `done`. |
| `marker_reporter.py` | Reports markers to scoring, see the README. |
| `wall_check.py` | Checks with HSV whether the wall straight ahead is white. While scanning, the robot moves on quickly from brick and wood walls, since markers are only on white walls. |

## How a robot works

1. **Pick a point.** The nearest free point by driving distance (BFS on the map). Points within 3 m of another robot's point cost 4 m extra, so the robots spread out.
2. **Drive there** with A*.
3. **Scan.** Turn towards each wall direction and wait 1.2 s, so ArUco gets several images. If `wall_check` says the wall is not white, move on after 0.2 s.
4. **Marker seen.** Turn towards it and stand still until it is reported. If it is too far or too oblique, first drive to a point 1.5 m in front of it.
5. **Big fire.** When ID 4 is reported, the robot that found it and the robot with the shortest drive into a 1.5 m circle around it go there.
6. **Second pass.** If every point is done and markers are still missing, plan new points for all walls again.

A wall only counts as seen after it has been in view twice (about 1 s) while the robot turns slower than 0.7 rad/s. ArUco only runs about 1.5 times per second, so a wall that just slides past should not be ticked off.

## Key choices

- **A\* instead of wall following.** Bug2 is for when the robot has no map. With the blueprint map the whole path can be planned in advance, and the robots drive the shortest way.
- **Limits for a "seen" wall** come from our own ArUco measurements: detected from 0.94 m to over 3.4 m and up to about 75°. The plan uses 2.3 m and 55° for margin.
- **Big fire:** scoring measures straight-line distance, so the helper is whichever robot drives into the 1.5 m circle fastest, whichever side of a wall that is.
- **Wall check is careful on purpose.** On map 2 the human marker is on a grey wall, so only clearly coloured walls (over 90 % of the image centre) count as not white.
- **Our own reporter instead of `marker_recognition`.** The course node combines an old image with the robot's current position. In our simulation that gave a typical error of about 0.4 m (often over 1 m when turning), against about 1 cm for ours.
- **Kept simple.** An earlier, more advanced version (reserve robots, splitting the map into regions, replanning from wall colour) scored about 30 points more in simulation, but was harder to explain and tune.

## Results

| Where | Setup | Result |
| --- | --- | --- |
| Own simulation, all 5 maps × 3 | 4 robots, 0.4 m/s, unreliable detection at long range | Mean 1251, lowest 1227, all markers found in 15/15 |
