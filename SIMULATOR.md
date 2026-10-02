# Running the simulator

This branch is the robot code **plus** `TeamCode/src/test/`, a JUnit suite that runs the real
OpModes — `RED AUTO`, `BLUE AUTO`, `Biobuzz TeleOp` — along with the real turret, shooter,
intake, vision and Pedro's Foresight algorithm, on your laptop against physics models of the
robot and the BIOBUZZ field. Only the drivetrain, the localizer and the camera are replaced.

Nothing here goes on the robot. The branch you upload is
`claude/turret-tracking-autonomous-opt-yv2u21`.

## What you need

- **Android Studio.** It supplies the JDK (Gradle 9.1 needs 17+) and the Android SDK, and it
  writes `local.properties` the first time you open the project. Without that file the build
  stops with "SDK location not found" — `local.properties` is gitignored, so it never arrives
  from a clone.
- **Internet on the first run only**, to fetch Pedro Pathing 3.0.0 from Maven Central. After
  that Gradle has it cached and `--offline` works.

## Run it

```bash
git fetch origin
git checkout claude/simulator

./gradlew :TeamCode:testDebugUnitTest          # macOS / Linux
gradlew.bat :TeamCode:testDebugUnitTest        # Windows
```

In Android Studio: right-click the `org.firstinspires.ftc.teamcode.sim` package in
`TeamCode/src/test/java` and choose **Run Tests**. Individual tests get a green arrow in the
gutter.

**15 tests, about two minutes.** It is slow on purpose: the robot code runs on wall-clock
timers, so a 30-second autonomous takes 30 seconds of real time. Output streams live, so you
can watch the AUTO state machine move through `TO_SCORE → SHOOT → TO_SCAN → … → PARK` and see
each shot's distance, RPM and whether it went in.

One class at a time:

```bash
./gradlew :TeamCode:testDebugUnitTest --tests "*TeleOpSimTest*"
```

## What you get

| File | What it is |
| --- | --- |
| `TeamCode/build/sim/auto_red.html`, `auto_blue.html` | **The replay.** Open in a browser: the field from above, the robot's path, the turret, every shot, and the Driver Station text at each instant. Play and scrub. |
| `TeamCode/build/sim/turret_tracking.html` | The turret holding the CELL while the robot spins and drives |
| `TeamCode/build/sim/*.csv` | The same runs as raw logs, for plotting |
| `TeamCode/build/reports/tests/testDebugUnitTest/index.html` | The JUnit report — which tests passed, and the full output of each |

The replays are self-contained HTML; you can send one to a teammate.

## Things worth knowing

- **Results move slightly between runs.** Real-time timers mean loop timing is never identical,
  so shot distances and tracking errors wobble a little. The assertions have margin for it.
- **If Gradle reports the test task `UP-TO-DATE`** and you wanted a fresh replay, add
  `--rerun-tasks`.
- **Don't merge this branch into the robot branch, in either direction.** The robot branch's
  history contains the commit that deleted `src/test`, so a merge tries to delete the simulator.
  To pull newer robot code in here, copy the source tree instead:

  ```bash
  git checkout claude/simulator
  git checkout claude/turret-tracking-autonomous-opt-yv2u21 -- TeamCode/src/main TeamCode/ROBOT_CONFIGURATION.md
  git commit -m "Sync robot code"
  ./gradlew :TeamCode:testDebugUnitTest
  ```

  That leaves `src/test/` and this branch's `TeamCode/build.gradle` (which adds JUnit and
  `unitTests.returnDefaultValues`) untouched.

## What the suite covers

| Test | What it checks |
| --- | --- |
| `AutoSimTest` | `RED AUTO` and `BLUE AUTO` end to end: 36 points each, 7 of 7 shots through the CELL mouth, HIVE tipped, GARDEN POLLEN collected, the opponent's NECTAR left alone, parked |
| `TeleOpSimTest` | The real TeleOp under a simulated driver: stick mapping, the turret holding the CELL while driving, intake and blocker, firing, and the hand-off from AUTO |
| `TurretTrackingSimTest` | Pointing error while spinning at 75 °/s and driving, and that a glitching position wire is rejected |
| `ShootingPhysicsTest` | The regression against the physics, shots dropping in from 18 to 78 in, wrong speeds missing, and that re-solving the table never moves a row |
| `PollenVisionSimTest` | The Limelight floor projection and field clustering, and that NECTAR is ignored |
| `PathHeadingTest` | Pins down Pedro's reversed `linear(a, b)` arguments |

Models: `SimRobot` (12 × 12 in mecanum with Mecanum's own power mixing, velocity lag, coasting
deceleration, odometry noise), `SimTurret` (two CR servos with dead zone and lag, two analog
wires with noise, wrap and injectable glitches), `SimShooter` (spin-up lag, RPM drop per ball),
`SimCamera` (field of view, frame rate, latency, pipeline switch time), `SimWorld` (the field,
ball handling, 3D ballistic shots through the tilted mouth, HIVE tipping, scoring by the rules)
and `SimReport` (the CSV and HTML replay).
