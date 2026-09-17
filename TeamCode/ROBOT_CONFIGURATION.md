# Robot Configuration

Hardware names the BIOBUZZ OpModes expect, and how the main subsystems fit together.

## OpModes on the Driver Station

| Name | Group | What it is |
| --- | --- | --- |
| `RED AUTO` / `BLUE AUTO` | Biobuzz | Full autonomous (`AutoBase`): preload shot, Limelight pollen seek + intake, return + shoot, park |
| `Biobuzz TeleOp` | Biobuzz | Match TeleOp; picks up the pose / turret angle / HIVE state saved by AUTO |
| `Shot Tuner` | Tuning | Builds `ShotTable` and tunes turret / shooter gains live |
| `Pollen Vision Test` | Tuning | Checks the Limelight floor projection against a real pollen piece |
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
  pickup (60 % speed, no brake at end), park.

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
  can start anywhere. With any other ratio (or both left NaN) the turret must face forward when
  an OpMode inits unless the angle was handed over from AUTO through `RobotState`.
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
- **Flight.** Point mass with gravity and quadratic air drag (5 in, 75 g ball, Cd 0.47),
  integrated in 2 ms steps from `LAUNCH_HEIGHT_IN`.
- **Hood.** The opening is tipped toward the shooter, so the ball must arrive descending. The
  exit angle follows the lob geometry `tan(exit) = 2h/d + tan(ENTRY_ANGLE_DEG)` (h = opening
  height above the launch, d = distance): near-vertical up close, flatter far away, clamped to
  the hood's range. Servo position comes from a two-point calibration
  (`HOOD_SERVO_AT_MIN_ANGLE` / `HOOD_SERVO_AT_MAX_ANGLE` at `HOOD_MIN_ANGLE_DEG` /
  `HOOD_MAX_ANGLE_DEG`).
- **Table.** For every distance from 24 to 144 in (6 in steps) the solver finds the exit speed
  that drops the ball *down* through the CELL opening (59.5 in) and converts it to RPM, plus
  time of flight for the turret's shoot-on-the-move lead.
- **Regression.** The RPM samples are fit with a cubic, `rpm = c0 + c1 d + c2 d² + c3 d³`,
  which `ShotTable.rpm()` uses (`USE_REGRESSION`). `ShotTable.describe()` prints the table,
  the coefficients and the fit error; the simulator's `ShootingPhysicsTest` does too.
- **CELL opening.** `Field.CELL_TILT_DEG` (30°) and `CELL_OPENING_RADIUS_IN` (placeholder 7 in,
  the radius the ball centre can pass) define the target the simulator scores against.

To tune on the robot: measure the ball, wheel, launch height and hood calibration into
`Ballistics`, then shoot from one known distance and adjust `EFFICIENCY_TRIM` until it drops
in; `ShotTable.regenerate()` (or a restart) rebuilds everything.

## Limelight pollen seeking (AUTO)

- Build one **Color** pipeline per pollen colour on the Limelight web UI and put its index in
  `vision/Pollen.java` (defaults: PURPLE 1, GREEN 2, YELLOW 3; rename to the real colours).
  Raise "max targets" so every blob is reported.
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

## AUTO waypoints

All in `Field.java`, written for RED and mirrored for BLUE by `Alliance.fromRed()` (the field is
point-symmetric). `RED_START`, `RED_SCORE`, `RED_SCAN`, `RED_PICKUP_FALLBACK` and `RED_PARK`
are **placeholders** until the field is measured. Shooting spots must be on the opening side of
the up-CELL; the auto refuses to fire from the wrong side (G417).

## Tuning order

1. AutoTune `1. Mecanum Directions`, `2. Pinpoint`, `3. Foresight`, paste into `Constants`.
2. `Turret.ENCODER_DIRECTION` / `POWER_DIRECTION`, then turret gains in `Shot Tuner`.
3. `ShotTable` rows with `Shot Tuner`.
4. Limelight pipelines, then camera mount numbers with `Pollen Vision Test`.
5. Field waypoints, then `PathProfiles` end constraints if paths stall or overshoot.

## Simulator (`TeamCode/src/test/java/.../sim`)

The real OpModes, turret, shooter, intake, Pedro Foresight and the pollen finder run on the
desktop JVM against a simulated robot. Run it with:

```
./gradlew :TeamCode:testDebugUnitTest
```

It takes about 90 s (the code uses wall-clock timers, so the 30 s autos run in real time) and
writes replays to `TeamCode/build/sim/*.html` (open in a browser: play / scrub, the field from
above, the turret and where it should point, pollen, shots, Driver Station telemetry) plus
matching CSVs.

| Test | What it proves |
| --- | --- |
| `AutoSimTest` | `RED AUTO` and `BLUE AUTO` end to end: preload volley scores, camera picks the biggest purple pile and ignores green, intake collects it, second volley, park |
| `TurretTrackingSimTest` | turret stays on the CELL while the robot spins and drives; a glitching position wire is flagged and ignored |
| `PollenVisionSimTest` | the pollen finder places a pile within an inch or two of where it is |
| `ShootingPhysicsTest` | the generated table and its regression drop balls into the CELL from 24 to 132 in; 20 % slow or 25 % fast misses |

Models: `SimRobot` (mecanum velocity lag, brake, odometry noise), `SimTurret` (two CR servos
with dead zone and lag, two analog wires with noise and wrap), `SimShooter` (flywheel lag,
RPM drop per ball; shots fly in 3D through `Ballistics` into the tilted CELL opening), `SimCamera` (Limelight FOV, frame rate,
latency, pipeline switch time), and ball handling in `SimWorld`. The robot code is swapped
onto the models only through `Constants.localizerFactory` / `drivetrainFactory` and
`PollenVision.CAMERA_FACTORY`.
