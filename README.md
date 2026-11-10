# DAT160 Search & Rescue – gruppe 8

Semesterprosjekt i DAT160: eit team av Turtlebot3-robotar i Gazebo skal finne fem ArUco-markørar (brann, menneske og stor brann) i ein labyrint på 10 minutt, rapportere posisjonane deira og møtast ved den store brannen.

## Oppsett

Repoet vert klona rett inn i `src` i ROS2-arbeidsområdet:

```bash
cd ~/ros2_ws/src
git clone https://github.com/h597236/dat160-SaR-Gr8.git
cd dat160-SaR-Gr8
git checkout <branch>
cd ~/ros2_ws
colcon build --packages-select search_and_rescue_comp_26
source install/setup.bash
```

Pakken krev kurspakkane `multi_robot_challenge_23`, `ros2_aruco` og `multi_robot_scoring` (med `scoring` og `scoring_interfaces`) i same arbeidsområde.

**Scoring krev ein ekstra plugin i world-filene.** Kopier world-filene frå `multi_robot_scoring/worlds` til `multi_robot_challenge_23/worlds` og bygg `multi_robot_challenge_23` på nytt. Elles finst ikkje `/gazebo/get_entity_state`, og scoring-noden vert ståande og vente.

## marker_reporter (felles)

`marker_reporter` er den same i alle branchane. Han les ArUco-deteksjonane frå alle robotane, reknar ut posisjonen til kvar markør i kartet og rapporterer kvar ID éin gong til scoring-noden.

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

Køyr han med lista over robotane:

```bash
ros2 run search_and_rescue_comp_26 marker_reporter --ros-args -p robots:="['tb3_0','tb3_1']"
```

## Arbeidsflyt

- Kvar person jobbar i sin eigen branch, laga frå `main`.
- Endringar i felles filer (som `marker_reporter.py`) går inn i `main` via ein pull request. Dei andre hentar dei inn med `git merge main` i sin branch.
- Personlege notat ligg i `docs/<namn>.md`, ikkje i denne README-fila, så det ikkje vert konfliktar når branchar vert slått saman.
