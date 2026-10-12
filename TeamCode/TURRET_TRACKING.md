# BIOBUZZ turret tracking

`Biobuzz TeleOp`, `BLUE AUTO`, and `RED AUTO` use one `FieldLocalizer` for both Pedro driving and turret aiming. It reads the configured **Pinpoint and two odometry pods**, including the Pinpoint's internal IMU. Pose and velocity update every OpMode loop; turret auto-aim starts at START and continues while driving, intaking, or waiting to shoot. Gamepad 2 A still provides the existing lock-forward override. The HIVE's tipped state is selected by the driver with Y on either gamepad; odometry cannot detect a HIVE tip.

## Position and reset

All field X/Y values are inches. Heading is counterclockwise radians, zero toward +X. The existing field convention is (0, 0) at the audience/red corner, +X toward the blue wall, +Y away from the audience. Turret angles are **clockwise degrees from robot forward**.

1. Measure the robot's actual starting tracking-center X/Y and heading and set `Field.RED_START`. Blue uses the 180-degree field rotation in `Alliance.fromRed`. The current `(48, 9, 90 degrees)` is a placeholder.
2. INIT: keep the robot stationary while Pinpoint calibrates; wait for `Localization: READY`. Gamepad 1 B selects red, X selects blue, and A toggles a saved pose. A saved pose restores its matching alliance, CELL selection, and turret reference.
3. During TeleOp, physically return the robot to that alliance's **start position AND heading**, stop, then hold **gamepad 2 X for one second**. The controller rumbles once. Release X before resetting again.
4. The reset corrects X, Y, and heading together, clears old velocity and turret control history, and waits for a fresh Pinpoint sample. It preserves the physical turret encoder zero and trim. It does not recalibrate the IMU during the match.

Code can anchor to any other measured landmark with `follower.setPose(new Pose(xIn, yIn, headingRad)); turret.resetTracking();`. Use the same robot tracking-center reference as the pod offsets. Resetting at an arbitrary unknown spot cannot recover an absolute field position.

The robot pose is an estimate from wheel/IMU motion, not an absolute position measurement. Wheel slip, lifting the robot, and calibration errors can accumulate drift. Re-anchor at a known pose when needed. Saved state survives OpMode changes only while the RC app is running and assumes neither robot nor turret was moved while tracking was stopped. Reject the saved pose at INIT if this assumption is false.

## What moving compensation does

`TurretAim` first finds the actual CELL distance and bearing from the turret pivot. It predicts the pose at the configured release delay and subtracts the velocity the ball inherits during its flight. Pivot velocity includes both field translation and rotation carrying an offset pivot around the robot center. Flight time and compensated range are solved together. Shooter RPM and hood use that same compensated range.

The turret feedforward rate follows the entire compensated solution, including robot yaw, changing flight time, and rotation of the pivot offset. Angle differences wrap across +/-180 degrees, while physical turret commands respect the configured cable limits. `isOnTarget()` requires both small angular error and matching turning rate.

The model assumes constant field velocity and turn rate during prediction and that the release point is at the configured pivot. It does not model unpredictable future driver acceleration, aerodynamic effects, or a muzzle significantly offset from that pivot. Measure the resulting shot error under real movement; changing code alone cannot guarantee a hit.

| Parameter | Where | What to measure |
| --- | --- | --- |
| Pod type, directions and offsets | `pedro/Constants.java` | Current values are 4-bar pods, X forward/Y reversed, offsets 8.6 and 3.86 **cm**; verify for this robot |
| Start pose and CELL aim coordinates | `Field.java` | Actual field/turret-center positions; `CELL_AIM_OFFSET = 15` is an approximation |
| Pivot forward/left offset | `Turret.PIVOT_FORWARD_IN`, `PIVOT_LEFT_IN` | Inches relative to the localizer's tracking center |
| Turret encoder scale/sign and motor sign | `Turret` | Real gearing and direction; verify positive angle and power turn right |
| Mechanical travel | `Turret.MIN_ANGLE_DEG`, `MAX_ANGLE_DEG` | Cable/mechanical travel; current +/-180 degrees is a configuration value |
| Position/rate response | `Turret.kP/kI/kD/kV/kS` | Stationary accuracy, settling, then moving tracking |
| Release delay | `Turret.RELEASE_LATENCY_S` | Measured delay to ball release, in seconds; default 0, accepted 0 to 0.5 |
| Flight-time model | `ShotTable` | Seconds from release to arrival at each distance |

`LEAD_GAIN = 1` enables flight compensation. Set it to 0 for geometric tracking tests (release-delay prediction still applies). Leave `RELEASE_LATENCY_S = 0` until measured. See the [goBILDA Pinpoint user guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001_user-guide.pdf) for hardware offset and calibration procedures.

## Add the angle and speed regressions later

All three polynomial arrays in `ShotTable` use ascending powers of **distance in inches**:

```text
output = coefficients[0] + coefficients[1] * distanceIn
       + coefficients[2] * distanceIn^2 + ...
```

| Output | Coefficients | Enable flag |
| --- | --- | --- |
| Flywheel RPM | `RPM_COEFFICIENTS` | `USE_RPM_REGRESSION` |
| Hood angle in degrees above horizontal | `HOOD_ANGLE_DEG_COEFFICIENTS` | `USE_HOOD_ANGLE_REGRESSION` |
| Ball flight time in seconds | `TIME_OF_FLIGHT_S_COEFFICIENTS` | `USE_TIME_OF_FLIGHT_REGRESSION` |

Set `REGRESSION_MIN_DISTANCE_IN` and `REGRESSION_MAX_DISTANCE_IN` to the measured domain before enabling a fit. A fit outside that domain is rejected. Angle fits additionally require measured `HOOD_CALIBRATION_ANGLE_DEG` and `HOOD_CALIBRATION_SERVO_POSITION` arrays: increasing angles, with servo positions monotonic in either direction. This handles the actual hood linkage; servo position is not a physical angle.

Until those fits arrive, the existing interpolated RPM, hood-servo-position, and flight-time tables remain available and are labeled **UNCALIBRATED STARTING VALUES**. Replace the rows with measurements before setting `TABLE_CALIBRATED = true`. The legacy tables clamp to their endpoint values; this does not establish accuracy outside the measured distances. `Biobuzz Shot Tuner` provides manual RPM/hood adjustment and logs distance, RPM, servo position, and physical hood angle once mapped. Flight time and release delay require a separate timing measurement, for example from recorded shots.

Turret yaw comes from field geometry, not the hood-angle regression. Gamepad trims are corrections on top of the solution and do not fix a wrong field pose.

## Telemetry and verification

Telemetry shows localization health, X/Y/heading, field velocity, actual versus compensated distance, CELL bearing, turret angle/rate errors, shot-model status, and loop time. Calibration, pod faults, bad/nonfinite samples, and old cached samples make automatic aiming unavailable. Invalid aim data produces no turret drive and no distance-based shooter RPM command. Manual intake/fire controls remain driver-operated. READY indicates that the software tolerances are met; it does not certify the shot calibration.

Run the repeatable desktop checks from the repository root:

```powershell
.\gradlew.bat :TeamCode:trackingChecks :TeamCode:compileDebugJavaWithJavac --console=plain
```

Before match use, verify on the robot: known 24/48-inch translations, 90/360-degree turns, a return to the measured start, both alliance/CELL combinations, the reset after moving, turret cable limits, and stationary followed by strafing/rotating shots. Record measured pose and shot errors. The desktop checks validate math and reset/fault behavior; physical accuracy still requires those measurements.
