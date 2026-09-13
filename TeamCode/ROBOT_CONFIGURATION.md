# Robot Configuration

This file records the Robot Controller hardware names expected by the active `states` OpModes after the archive, auto-speed, and turret updates.

## Active OpModes

Only the OpModes in `org.firstinspires.ftc.teamcode.states` should appear on the Driver Station:

- `Blue TeleOp States`
- `Red TeleOp States`
- `Blue CLOSE AUTO STATE`
- `BLUE FAR AUTO STATE`
- `RED CLOSE AUTO STATE`
- `RED FAR AUTO STATE`

All other OpModes were moved under `org.firstinspires.ftc.teamcode.archive` and disabled.

## Hardware Names

Configure these names in the Robot Controller configuration:

| Name | Type | Used by |
| --- | --- | --- |
| `frontLeftMotor` | DC motor | TeleOp drive |
| `frontRightMotor` | DC motor | TeleOp drive |
| `backLeftMotor` | DC motor | TeleOp drive |
| `backRightMotor` | DC motor | TeleOp drive |
| `intakeMotor` | DC motor | Intake / claw subsystem |
| `shooterMotor` | DC motor with encoder capability (`DcMotorEx`) | Shooter output and RPM feedback |
| `shooterMotor2` | DC motor with encoder capability (`DcMotorEx`) | Shooter output and RPM feedback |
| `turretMotor` | DC motor with encoder capability (`DcMotorEx`) | Turret rotation |
| `turretEncoder` | Analog input | Turret absolute angle feedback |
| `rotateServo` | Servo | TeleOp mechanism |
| `blockServo` | Servo | Blocker / claw subsystem |
| `aimServo` | Servo | Shooter aim |
| `pinpoint` | GoBilda Pinpoint | Odometry / heading |
| `limelight` | Limelight 3A | AprilTag targeting and far-zone ball lane selection |

## Turret Notes

- The turret actuator is now `turretMotor`, not `turretServo`.
- Turret angle comes from the dedicated analog input named `turretEncoder`.
- The active turret code enforces a software hard limit from `-150` to `+150` degrees relative to the turret's starting position.
- `turretMotor` is set to `RUN_WITHOUT_ENCODER` with `BRAKE` zero-power behavior.
- If the turret tracks in the wrong direction, reverse the sign of the turret encoder integration in `SetTurretWithIMU` before tuning PID constants.
- If the turret motor moves the wrong direction, reverse `turretMotor` in the Robot Controller configuration before tuning PID constants.
- Current turret gear ratio passed by active states: `2.59375`.

## Shooter Notes

- Both shooter motors are still powered together.
- Distance-based shooter targets use the old regression: `RPM = distance * 0.010472 + 3561.36`.
- Shooter RPM feedback averages `shooterMotor.getVelocity()` and `shooterMotor2.getVelocity()` when both encoders are usable.
- If only one shooter encoder is usable, feedback falls back to that encoder. If both are present, Driver Station telemetry shows each motor's RPM separately.
- If neither shooter encoder is live, shooter control uses direct motor power. The default no-encoder shooter power is `0.7`, and D-pad up/down in TeleOp adjusts it.

## Far-Zone Ball Targeting

- Far autos use Limelight pipeline `6` for AprilTag scoring and switch to pipeline `1` before each far-zone ball pickup.
- Pipeline `1` is expected to be an object detector for purple and green balls. If detector class names include `purple` or `green`, only those named detections are counted. If class names are different, detections are treated as balls only when they pass confidence and target-area thresholds.
- Ball detections are mapped to the closest existing pickup Y lane by comparing `DetectorResult.getTargetXDegrees()` against the expected horizontal angle from the current robot pose to each lane pose. Red lanes are approximately `4.75`, `24`, and `42`; blue lanes are approximately `-4.75`, `-24`, and `-42`.
- If no usable detections are available, or if lanes tie, the auto falls back to the existing cycle lane.
- Driver Station telemetry reports the active ball pipeline, selected lane, raw and usable detection counts, lane counts, and the latest detector sample.

## Autonomous Speed Notes

The four active autonomous state files use two Pedro path-end profiles:

- Score paths: velocity `5.0`, translational error `1.5`, heading error `Math.toRadians(3)`, timeout `0.75`
- Transit / pickup / park paths: velocity `7.0`, translational error `2.0`, heading error `Math.toRadians(4)`, timeout `0.6`
- Score path max power: `1.0`
- Pickup path max power: `0.85`
- Park path max power: `0.85`

Several fixed waits were shortened to reduce dead time while keeping settle/release gates in place, and pickup/scoring path launches now go through helper methods so power caps are applied consistently on every cycle.

The close-side autos now start the next pickup path immediately after the post-score claw reset. Park states are active again in the autonomous state machines and route through the park speed helper instead of launching the park path directly.
