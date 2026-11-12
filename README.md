# DAT160 Search & Rescue – gruppe 8

Semesterprosjekt i DAT160: eit team av Turtlebot3-robotar i Gazebo skal finne fem ArUco-markørar (brann, menneske og stor brann) i ein labyrint på 10 minutt, rapportere posisjonane deira og møtast ved den store brannen.

Robotane reknar ut frå blueprint-kartet kvar dei må stå for at kameraet skal sjå alle veggane, fordeler punkta mellom seg og køyrer dit med A*. Ein leiarnode (`mission`) styrer kva kvar robot gjer, og stoppar alt når alle fem markørane er rapporterte og to robotar har vore ved den store brannen.

## Krav

ROS2 Foxy på Ubuntu 20.04, med Gazebo og Turtlebot3.

Desse kurspakkane må liggje i same arbeidsområde (`~/ros2_ws/src`):

| Pakke | Kva me brukar frå ho |
| --- | --- |
| `multi_robot_challenge_23` | Blueprint-karta (`maps/`) og robotmodellen (`urdf/`) |
| `ros2_aruco` og `ros2_aruco_interfaces` | `aruco_node` og meldingstypen `ArucoMarkers` |
| `multi_robot_scoring` (`scoring` og `scoring_interfaces`) | Servicetypen `SetMarkerPosition`, og scoring-noden når han vert starta med `scoring:=true` |

`scoring_interfaces` må alltid vere bygd, sidan nodane våre brukar servicetypen. Sjølve scoring-noden treng berre vere der viss de startar han sjølve.

I tillegg trengst `nav2_map_server`, `nav2_lifecycle_manager`, `cv_bridge`, `python3-opencv` og `python3-numpy`.

## Oppsett

```bash
cd ~/ros2_ws/src
git clone https://github.com/h597236/dat160-SaR-Gr8.git
cd ~/ros2_ws
colcon build --packages-select scoring_interfaces scoring search_and_rescue_comp_26
source install/setup.bash
```

World-filene ligg i pakken vår (`worlds/`). Dei har pluginen `libgazebo_ros_state.so`, som scoring treng for å sjå kvar robotane er.

## Køyre

Terminal 1, start simuleringa (byt `x` med kart 1–5):

```bash
ros2 launch search_and_rescue_comp_26 sim.launch.py world:=x
```

Terminal 2, start robotteamet:

```bash
ros2 launch search_and_rescue_comp_26 team.launch.py speed:=0.4
```

Scoring-noden er av som standard, sidan han vanlegvis vert starta separat (til dømes av læraren). For å starte han saman med teamet når de testar:

```bash
ros2 launch search_and_rescue_comp_26 team.launch.py speed:=0.4 scoring:=true
```

Robotane ventar til `/set_marker_position` finst før dei køyrer. Utan ein scoring-node står dei altså stille.

| Argument | Standard | Kva |
| --- | --- | --- |
| `world` (sim) | `1` | Kart 1–5 |
| `robots` (sim og team) | `4` | Tal robotar, 1–5, same tal i begge |
| `speed` (team) | `0.26` | Fart fram (m/s) |
| `scoring` (team) | `false` | Start scoring-noden saman med teamet |
| `use_wall_check` (team) | `true` | Hopp raskare over vegger som ikkje er kvite når roboten scannar |
| `mission` (team) | `true` | `false` = berre navigator og reporter, for å sende mål for hand |

## Nodar

| Fil | Kva ho gjer |
| --- | --- |
| `navigator.py` | Éin per robot. Planlegg veg med A* på kartet og køyrer langs han. Einaste noden som sender `cmd_vel`. |
| `coverage.py` | Ikkje ein node. Finn punkta som til saman ser alle veggane (grådig set cover). |
| `mission.py` | Leiaren. Tilstandsmaskin per robot: `explore`, `to_vp`, `scan`, `inspect`, `approach`, `go_big_fire`, `wait_big_fire`, `done`. |
| `marker_reporter.py` | Reknar ut kvar markørane er i kartet og rapporterer kvar ID éin gong til scoring. |
| `wall_check.py` | Ser med HSV om veggen rett framfor er kvit eller ikkje. |

### marker_reporter

- Les `/<robot>/aruco_markers` direkte og slår opp TF **på tidspunktet biletet vart teke**. Kursnoden `marker_recognition` brukar posisjonen roboten har *no*, og det gir feil posisjon når roboten bevegar seg.
- Brukar berre målingar innanfor 3,0 m og 65° synsvinkel. Grensene kjem frå testane våre av ArUco-deteksjonen.
- Tek snittet av 5 målingar og rapporterer med z = 0, sidan scoringa samanliknar i 3D mot z = 0.
- Vert same ID avvist 3 gonger, vert ho rekna som allereie rapportert (til dømes etter at noden er starta på nytt).

| Topic | Type | Retning | Innhald |
| --- | --- | --- | --- |
| `/<robot>/aruco_markers` | `ros2_aruco_interfaces/ArucoMarkers` | inn | Markørar i kamerabiletet |
| `/set_marker_position` | `scoring_interfaces/SetMarkerPosition` | service, ut | Rapport til scoring |
| `/reported_markers` | `std_msgs/Int8MultiArray` | ut, kvart sekund | ID-ar scoringa har godteke |
| `/big_fire_position` | `geometry_msgs/Point` | ut, kvart sekund | Posisjonen til ID 4 når han er godteken |

## RViz

`sim.launch.py` opnar RViz med `rviz/team.rviz`. Der ser du vegen til kvar robot og dekningspunkta (blå = att, grå = ferdig).
