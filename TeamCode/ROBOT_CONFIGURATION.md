# Robot Configuration

Hardware names the BIOBUZZ OpModes expect, and how the main subsystems fit together.

## OpModes on the Driver Station

| Name | Group | What it is |
| --- | --- | --- |
| `RED AUTO` / `BLUE AUTO` | Biobuzz | Full autonomous (`AutoBase`): 3 POLLEN tip the HIVE, find and intake the GARDEN POLLEN with the Limelight, re-score into the new up CELL, PARK |
| `Biobuzz TeleOp` | Biobuzz | Match TeleOp; picks up the pose / turret angle / HIVE state saved by AUTO |
| `Shot Tuner` | Tuning | Builds `ShotTable` and tunes turret / shooter gains live |
| `Pollen Vision Test` | Tuning | Checks the Limelight floor projection against a real POLLEN / NECTAR piece |
| AutoTune (browser) | Tuning | Pedro AutoTune procedures in `pedro/Tuning.java`, at http://192.168.43.1:10158 |

## Hardware names

| Name | Type | Used by |
| --- | --- | --- |
| `frontLeftMotor` | DC motor | Drive |
| `frontRightMotor` | DC motor | Drive |
| `backLeftMotor` | DC motor | Drive |
| `backRightMotor` | DC motor | Drive |
| `turretServo`, `turretServo2` | Continuous rotation servos (`CRServo`) | Turret rotation, both driving the same ring; both must be programmed to continuous / "infinite turn" mode. Set `Turret.SERVO2_DIRECTION` to -1 if the second is mounted mirrored |
| `turretEncoder`, `turretEncoder2` | Analog inputs | The two servos' position wires: 0 to `Turret.ANALOG_MAX_VOLTAGE` per servo revolution. Both are used: averaged while they agree, the glitched one ignored when they don't |
| `intakeMotor`, `transferMotor` | DC motors | Intake and transfer (`Intake`): both run while intaking and while shooting, off otherwise |
| `shooterMotor`, `shooterMotor2` | DC motors with encoders | Flywheel; RPM feedback averages both encoders |
| `aimServo` | Servo | Shooter hood |
| `blockServo` | Servo | Blocker between transfer and flywheel: engaged while intaking (balls stack behind it), released while shooting |
| `pinpoint` | goBILDA Pinpoint (I2C) | Odometry: two dead wheels + the Pinpoint's IMU |
| `imu` | Control Hub IMU (built in) | Heading backstop if the Pinpoint faults (`FusedPinpointLocalizer`) |
| `limelight` | Limelight 3A | Pollen colour blobs in AUTO (`vision/PollenVision`) |

## Localization (dead wheels + IMU)

`pedro/Constants.java` builds the Pedro Follower from three custom pieces:

- `FusedPinpointLocalizer` wraps Pedro's Pinpoint localizer. The Pinpoint fuses its two dead
  wheels with its own IMU, so the pose and heading the turret uses come from dead wheels + IMU.
  The Control Hub IMU is read every 100 ms only to track the constant offset between the two
  headings; if the Pinpoint reports a fault (pod unplugged, IMU runaway, bad read) the heading
  switches to hub IMU + offset and the position freezes at the last good value, so the turret
  keeps tracking the goal. It also keeps 1.5 s of pose history so Limelight detections are
  placed with the pose from when the frame was captured.
- `CompensatedDrivetrain` is the stock mecanum drivetrain with battery voltage compensation
  (`Battery.NOMINAL_VOLTAGE` / measured, capped at 0.85 to 1.2).
- `PathProfiles` are per-path Foresight overrides: score (stop precisely), transit (fast),
  pickup (60 % speed, no brake at end), park. It also owns `straight(from, to, profile)`, which the
  auto builds every straight path with.

**Pedro 3.0.0 heading quirk.** `Path.linear(a, b)` holds `b` at the start of the path and `a`
at the end: the arguments read backwards from their names. Written the obvious way, every path
drives the robot back to the heading it started at, which is invisible on a turret robot until
the camera or the intake has to point somewhere. `PathProfiles.straight()` passes them in the
order the library actually wants, and `PathHeadingTest` in the simulator fails loudly if a
later Pedro release swaps them back.

**Path end vs heading.** Pedro reports a path finished at its parametric end, which can be well
before the heading has settled (translation converges to hundredths of an inch first). The
shooting spots do not care, because the turret aims itself; the scan pose and the pickup
approach do, so `AutoBase.followTo(..., true)` also waits for the heading
(`PATH_HEADING_TOLERANCE_DEG`, `PATH_SETTLE_TIMEOUT_S`).

Set `FusedPinpointLocalizer.LOGO_FACING` / `USB_FACING` to how the Control Hub is mounted; they
only matter for the fallback heading.

## Turret

- Field-relative aim from odometry only, no camera. Angles are degrees from robot forward,
  positive = right.
- Actuators are two servos in continuous-rotation mode driving the same ring, so `setPower()`
  commands speed. Check the two servos turn the ring the same way before anything else: with
  power applied they must not fight each other (`Turret.SERVO2_DIRECTION`).
- Feedback is both servos' analog position wires. Each wraps once per servo revolution; the
  code unwraps each, divides by `Turret.GEAR_RATIO` (servo revs per turret rev), and averages
  them. If they differ by more than `Turret.WIRE_AGREE_DEG` the wire nearer the predicted angle
  is used and telemetry shows "WIRES DISAGREE". Flip `ENCODER2_DIRECTION` if the second wire
  counts the other way (mirrored servo).
- Control is a cascade with the speed under PID: the position loop (`POS_kP`, `POS_kI`) turns
  angle error plus the aim point's motion into a target speed, capped at `MAX_VELOCITY_DEG_S`;
  the velocity PID (`kP`, `kI`, feedforward `kV`, `kS`) holds that speed against the measured
  turret speed. "Turret vel / cmd" in telemetry shows measured vs commanded speed.
- Starting position: with `GEAR_RATIO = 1` set `Turret.FORWARD_RAW_DEG` and `FORWARD_RAW2_DEG`
  to the "Turret raw 1 / 2" telemetry values read with the turret facing forward, and the turret
  can start anywhere - the wires are then absolute, and both OpModes say so on the init screen
  ("absolute from the position wires"). TELEOP keeps that reading instead of AUTO's saved angle,
  so nudging the ring between OpModes costs nothing. With any other ratio (or both left NaN) the
  turret must face forward when an OpMode inits, unless the angle was handed over from AUTO
  through `RobotState`, and the init screen says "must be FACING FORWARD now".
- Tuning order in `Shot Tuner`: measure `kV` first (full power in MANUAL mode, read "Turret
  vel", kV = 1 / that speed in deg/s), then raise vel `kP` until the measured speed follows the
  commanded speed without oscillating, then pos `kP` until it snaps to target without
  overshoot, then the small `kI` terms for the last degree.
- Software limits `Turret.MIN_ANGLE_DEG` / `MAX_ANGLE_DEG` (±180). The turret pins at a limit
  instead of swinging 360° when the goal is just past it.
- PIDF with velocity feedforward: the turret cancels the robot's own spin and drive so it stays
  locked on while moving, and leads the shot by the ball's time of flight (`Turret.LEAD_GAIN`,
  `ShotTable.TIME_OF_FLIGHT_S`).
- Settle hysteresis: it stops commanding inside `DEADBAND_DEG` and only re-engages past
  `UNSETTLE_DEG`, so it does not buzz on target.
- If the turret angle reads the wrong way (turn it right by hand, the angle should go up), flip
  `Turret.ENCODER_DIRECTION`; if positive power turns it left, flip `Turret.POWER_DIRECTION`.
  Do that before touching the gains.
- `Turret.PIVOT_FORWARD_IN` / `PIVOT_LEFT_IN`: turret pivot relative to the odometry centre.

## Intake (`Intake.java`)

| Mode | Intake + transfer motors | Blocker |
| --- | --- | --- |
| `OFF` | off | engaged |
| `INTAKE` | on (`INTAKE_POWER`, `TRANSFER_POWER`) | engaged |
| `SHOOT` | on (`SHOOT_*_POWER`, slower so the flywheel recovers) | released |

Shooting is intaking straight through: the same motors feed the flywheel once the blocker
drops. Servo positions are `BLOCKER_ENGAGED` / `BLOCKER_RELEASED`.

## Shooter

- Flywheel power = voltage-compensated feedforward (`kV * RPM + kS`) plus PI; full power when
  far below target to recover after a ball.
- If neither flywheel encoder is live the shooter runs on feedforward alone and telemetry says
  so.

### Shot physics and the RPM regression (`Ballistics.java`, `ShotTable.java`)

The shot table is not hand-typed: it is generated from physics when the code loads.

- **Flywheel to ball.** One wheel against a fixed hood: no-slip exit speed is half the wheel
  surface speed. Grip depends on compression, `1 - exp(-COMPRESSION_MM / FULL_GRIP_COMPRESSION_MM)`;
  at the design compression of **4 mm** that is 74 % of ideal, so exit speed = 0.37 × surface
  speed. `EFFICIENCY_TRIM` (measured / predicted) is the one number to fit on the robot from a
  chronograph or one calibrated distance.
- **Flight.** Point mass with gravity and quadratic air drag (POLLEN: 2.8 in, 24.9 g, Cd 0.47),
  integrated in 2 ms steps from `LAUNCH_HEIGHT_IN`. Gamepad 2 B in TeleOp switches the table
  to NECTAR (3.62 in, 41.3 g) and back.
- **Hood.** The opening is tipped toward the shooter, so the ball must arrive descending. The
  exit angle follows the lob geometry `tan(exit) = 2h/d + tan(ENTRY_ANGLE_DEG)` (h = opening
  height above the launch, d = distance): near-vertical up close, flatter far away, clamped to
  the hood's range. Servo position comes from a two-point calibration
  (`HOOD_SERVO_AT_MIN_ANGLE` / `HOOD_SERVO_AT_MAX_ANGLE` at `HOOD_MIN_ANGLE_DEG` /
  `HOOD_MAX_ANGLE_DEG`).
- **Table.** For every distance from 18 to 90 in (6 in steps; the mouth is at most ~80 in from
  any legal spot in its half) the solver finds the exit speed that drops the ball *down* through
  the mouth centre (59.55 in) and converts it to RPM, plus time of flight for the turret's
  shoot-on-the-move lead.
- **Regression.** The RPM samples are fit with a cubic, `rpm = c0 + c1 d + c2 d² + c3 d³`,
  which `ShotTable.rpm()` uses (`USE_REGRESSION`). `ShotTable.describe()` prints the table,
  the coefficients and the fit error; the simulator's `ShootingPhysicsTest` does too.
- **CELL mouth.** The up CELL's mouth is a 20 × 14 in rectangle 21.46 in along the 30° arm
  from the pivot (centre 59.55 in up, 18.6 in from the pivot line), facing 30° above horizontal
  toward the shooter (`Field.cellMouth`). The simulator scores a ball only if it crosses that
  plane going in, inside the rectangle by its radius.

To tune on the robot: measure the ball, wheel, launch height and hood calibration into
`Ballistics`, then shoot from one known distance and adjust `EFFICIENCY_TRIM` until it drops
in; `ShotTable.regenerate()` (or a restart) rebuilds everything.

## BIOBUZZ game facts the code relies on (Competition Manual V1 / TU01, Field Setup Guide V1.0)

| Item | Value | Where in the code |
| --- | --- | --- |
| POLLEN | yellow, 2.8 in, 24.9 g, 40 per match, either alliance | `Field.POLLEN_*`, `GamePiece.POLLEN` |
| NECTAR | red / blue, 3.62 in, 41.3 g, 8 per alliance, own alliance only (G408) | `Field.NECTAR_*`, `GamePiece.*_NECTAR` |
| Setup | 4 POLLEN preloaded per robot, 4 per GARDEN, 4 per FLOWER; 3 NECTAR in each up CELL | `Field.PRELOAD_POLLEN`, `RED_GARDEN_POLLEN`, `NECTAR_STAGED_IN_UP_CELL` |
| HIVE | pivots 43.95 in up, 25.5 in apart, arm 30°, CELL mouth 20 × 14 × 12 in, lip 53.5 / top 65.6 | `Field.HIVE_*`, `CELL_*`, `cellMouth()` |
| HIVE TIP | 8 POLLEN or 3 POLLEN + 3 NECTAR (so 3 POLLEN tip the starting CELL); 20 points; other CELL comes up | `Field.TIP_POLLEN_EQUIVALENTS`, `AutoBase.TIP_VOLLEY_BALLS` |
| Start | touching the alliance wall, inside an 18 in cube; red up CELL faces the audience, blue faces the rear | `Field.RED_START`, `startingUpCell()` |
| LOADING ZONE | 23 × 11 in against the alliance wall, rear half; PARK = 5 | `Field.RED_LOADING_ZONE`, `RED_PARK` |
| GARDEN | 23 × 2 in tape from the alliance corner along the audience (red) / rear (blue) wall; 1 point per element left | `Field.RED_GARDEN` |
| Points | LEAVE 3, PARK 5 (AUTO and TELEOP), TIP 20, element left in up CELL 2, GARDEN 1 | `SimWorld.autoPoints()` |
| Limits | possession 4 (G407), only LAUNCH into the up CELL (G417) | `Field.POSSESSION_LIMIT`, `AutoBase` |

Coordinates: the manual's frame has its origin at field centre, +Y from the red wall toward
the blue wall, +X toward the audience; Pedro here uses inches from the red/audience corner
(`Field.fromOfficial`). Robot size in the code is your 12 × 12 in footprint
(`Field.ROBOT_SIZE_IN`).

## Limelight pollen seeking (AUTO)

- Build one **Color** pipeline per piece on the Limelight web UI and put its index in
  `vision/GamePiece.java` (defaults: POLLEN yellow 1, RED_NECTAR 2, BLUE_NECTAR 3). Raise
  "max targets" so every blob is reported.
- `PollenVision` projects each blob to the floor from the camera mount
  (`CAMERA_FORWARD_IN`, `CAMERA_LEFT_IN`, `CAMERA_HEIGHT_IN`, `CAMERA_PITCH_DEG`,
  `CAMERA_YAW_DEG`, `POLLEN_HEIGHT_IN`), places it on the field with the capture-time pose,
  and merges it into clusters whose weight is the accumulated blob area (decaying by half each
  second unseen). The heaviest cluster is where the most of that pollen is.
- `AutoBase.TARGET_POLLEN` picks the colour. After scanning, the auto drives to
  `STANDOFF_IN` short of the heaviest cluster within `Field.MAX_POLLEN_CHASE_IN` of the scan
  pose, turns the intake to it (`INTAKE_HEADING_OFFSET`: 0 front, π back), runs the intake and
  pushes `PUSH_THROUGH_IN` past it. No cluster → `Field.RED_PICKUP_FALLBACK`.
- Verify the geometry with `Pollen Vision Test` before a match.

## AUTO plan and waypoints

`AutoBase`: start on the alliance wall with 4 POLLEN → drive to `RED_SCORE_AUDIENCE` (32 in
from the up CELL's mouth) → fire 3 POLLEN → the HIVE tips (the auto assumes it and flips the
turret to the FAR CELL) → drive to the GARDEN corner (`RED_SCAN`), find the 4 POLLEN with the
Limelight, intake them → drive around to `RED_SCORE_REAR` (the new up CELL faces the rear)
and fire everything → PARK in the LOADING ZONE (`RED_PARK`). Expected AUTO: LEAVE 3 + TIP 20
+ PARK 5 + 2 per POLLEN left in the rear CELL.

Waypoints are in `Field.java`, written for RED and mirrored for BLUE by `Alliance.fromRed()`.
They are placed from the published geometry but not yet driven on a real field: expect to
nudge them. Shooting spots must be on the opening side of the up CELL; the auto refuses to
fire from the wrong side (G417).

## Tuning order

1. AutoTune `1. Mecanum Directions`, `2. Pinpoint`, `3. Foresight`, paste into `Constants`.
2. `Turret.ENCODER_DIRECTION` / `POWER_DIRECTION`, then turret gains in `Shot Tuner`.
3. `ShotTable` rows with `Shot Tuner`.
4. Limelight pipelines, then camera mount numbers with `Pollen Vision Test`.
5. Field waypoints, then `PathProfiles` end constraints if paths stall or overshoot.

## Simulator (on the `claude/simulator` branch, NOT in the robot code)

This branch holds only code that goes on the robot. The desktop simulator that was used to
verify all of it lives on the **`claude/simulator`** branch, under `TeamCode/src/test/`.

It is a JUnit suite that runs the real OpModes, turret, shooter, intake, vision and Pedro
Foresight on a laptop against physics models of the robot and the BIOBUZZ field. Nothing in it
is needed to build or run the robot, and Android never packages `src/test` into the APK, but it
is kept off this branch so what you upload is only robot code. To use it:

```
git checkout claude/simulator
./gradlew :TeamCode:testDebugUnitTest      # 15 tests, ~2 min, replays in TeamCode/build/sim/*.html
```

The robot code has three small seams so the simulator can stand in for hardware, and they are
harmless (and useful) on the real robot:

| Seam | What it is for |
| --- | --- |
| `Constants.localizerFactory` / `drivetrainFactory` | swap the Pinpoint and mecanum for models |
| `pedro.PoseHistory` | the pose at an earlier instant, for camera latency |
| `vision.PollenCamera` + `PollenVision.CAMERA_FACTORY` | swap the Limelight for a virtual one |

What the suite proved, on the field geometry in `Field.java`:

| Test | Result |
| --- | --- |
| `AutoSimTest` | `RED AUTO` and `BLUE AUTO`, 30 s each: **36 AUTO points** (LEAVE 3 + TIP 20 + PARK 5 + 8 left in the CELL), 7 of 7 shots in, HIVE tipped once, GARDEN POLLEN collected, an opponent NECTAR correctly ignored |
| `ShootingPhysicsTest` | the cubic regression fits the physics to 0.03 %; POLLEN drops through the CELL mouth from 18 to 78 in; 20 % slow or 25 % fast misses; NECTAR needs no separate table |
| `TurretTrackingSimTest` | settled error 0.14 deg, 95th percentile 3.6 deg while spinning at 75 deg/s and driving; a glitching position wire moves the turret 0.7 deg and is flagged |
| `PollenVisionSimTest` | the GARDEN line is located 0.8 in from truth and NECTAR is ignored |
| `TeleOpSimTest` | `Biobuzz TeleOp` under a simulated driver: the stick mapping drives forward / strafes right / turns clockwise as labelled, the turret holds the CELL to 3.2 deg while the driver drives, the trigger and bumper work the intake and blocker, holding fire from the shooting spot scores, and the AUTO hand-off restores the pose and up CELL |
| `PathHeadingTest` | pins down Pedro's reversed `linear(a, b)` arguments |

Models behind it: `SimRobot` (12 in mecanum, velocity lag, braking, odometry noise),
`SimTurret` (two CR servos with dead zone and lag, two analog wires with noise, wrap and
injectable glitches), `SimShooter` (flywheel lag, RPM drop per ball), `SimCamera` (Limelight
field of view, frame rate, latency, pipeline switch time), `SimWorld` (fake HardwareMap,
POLLEN and NECTAR on the tiles, intake and blocker ball handling, 3D ballistic shots into the
tilted CELL mouth, HIVE tipping with its 0.88 s swing, AUTO points counted like the rules) and
`SimReport` (CSV plus a self-contained HTML replay with play and scrub).
