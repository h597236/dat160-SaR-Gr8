# Thomas si løysing

Robotane reknar ut frå blueprint-kartet kvar dei må stå for at kameraet skal sjå alle veggane, fordeler punkta mellom seg og køyrer dit med A*. Ein leiarnode styrer kva kvar robot gjer, og stoppar alt når alle fem markørane er funne og big fire-møtet er gjort.

## Nodar

| Fil | Kva ho gjer |
| --- | --- |
| `navigator.py` | Éin per robot. Planlegg veg med A* på kartet og følgjer han med go-to-point frå Bug2 (`fix_yaw`, `go_straight`). Lidaren oppdagar hinder som ikkje står i kartet. Einaste noden som sender `cmd_vel`. |
| `coverage.py` | Ikkje ein node. Finn veggflatene som må sjåast, kva kvart mogleg punkt ser (1,0–2,3 m, under 55°, fri sikt), og vel punkt grådig til 95 % er dekt (set cover). |
| `mission.py` | Leiaren. Tilstandsmaskin per robot: `explore`, `to_vp`, `scan`, `inspect`, `approach`, `go_big_fire`, `wait_big_fire`, `done`. |
| `marker_reporter.py` | Felles for alle, sjå README i `main`. |
| `wall_colour.py` | Valfri. Kamera og lidar finn veggar som tydeleg er tre eller mur. Markørane sit berre på kvite veggar, så mission hoppar over dei. Grå eller usikre veggar vert aldri melde. |

## Køyre

```bash
ros2 launch search_and_rescue_comp_26 sim.launch.py world:=1
ros2 launch search_and_rescue_comp_26 team.launch.py
```

`team.launch.py` startar òg scoring-noden. Robotane reknar ut planen og ventar til scoring køyrer, sidan scoring-klokka startar med ein gong når det er fleire enn 2 robotar.

| Argument | Standard | Kva |
| --- | --- | --- |
| `world` (sim) | `1` | Kart 1–5 |
| `robots` (sim og team) | `4` | Tal robotar, 1–5, same tal i begge |
| `speed` | `0.26` | Fart fram (m/s) |
| `turn_speed` | `1.8` | Maks sving mellom punkt (rad/s) |
| `face_turn_speed` | `1.0` | Sving når roboten ser rundt seg ved eit punkt |
| `scan_all` | `true` | Sjå på alle veggane ved eit punkt |
| `far_robots` | `2` | Kor mange av dei siste robotane som startar med punkta lengst unna |
| `use_wall_colour` | `false` | Start kvit-vegg-deteksjonen |
| `reserves` | `0` | Robotar som ventar som big fire-hjelparar |
| `scoring` | `true` | `false` viss scoring vert starta separat |
| `mission` | `true` | `false` = berre navigator og reporter, for å sende mål for hand |

RViz (`rviz/team.rviz`) viser vegen til kvar robot, dekningspunkta (blå = att, grå = ferdig) og veggar meldt som ikkje kvite (oransje).

## Viktige val

- **A\* i staden for veggfølging.** Bug2 passar når roboten ikkje har kart. Med blueprint-kartet kan heile vegen planleggjast på førehand.
- **Grensene for «sett vegg»** kjem frå eigne målingar av ArUco: sett frå 0,94 m til over 3,4 m og opp til om lag 75°. Planen brukar 2,3 m og 55° for margin.
- **Big fire:** scoringa måler i luftlinje. Hjelparen er den som raskast køyrer inn i 1,5 m-sirkelen rundt markøren, på kva side av veggen det enn er.
- **Kvit-vegg-deteksjonen er forsiktig med vilje.** På kart 2 sit menneske-markøren på ein grå vegg, så berre tydeleg farga veggar vert melde.

## Resultat

| Kor | Oppsett | Resultat |
| --- | --- | --- |
| Gazebo, kart 1 | 4 robotar, 0,26 m/s (tidleg versjon) | 1205 poeng |
| Gazebo, alle 5 kart × 2 | 4 robotar, 0,5 m/s | Alle markørar funne i første runde |
| Eigen simulering, alle 5 kart × 3 | 4 robotar, 0,3 m/s, upåliteleg deteksjon på lang avstand | Snitt 1203, lågast 1147, alle funne |
