package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FusedPinpointLocalizer;

/**
 * Gamepad 1
 *   sticks: drive (robot-centric)
 *   hold right bumper: fire
 *   right trigger: intake   left trigger: stop intake   left bumper: close blocker + stop intake
 *   Y: HIVE tipped, aim at the other CELL
 *   dpad right/left: turret trim +/-5 deg   dpad up/down: +/-10 deg (+ = right)
 *   back: clear turret trim
 * Gamepad 2
 *   right trigger: intake
 *   Y: HIVE tipped, aim at the other CELL
 *   A: toggle turret auto-aim / lock forward (fallback if odometry is off)
 *   dpad right/left: turret trim +/-3 deg   bumpers: +/-10 deg
 *   dpad up/down: flywheel RPM trim +/-50
 *   hold X for 1 s: robot is back on its start spot, reset odometry to the start pose
 * Init
 *   gamepad 1 X = blue, B = red, A = ignore / use the position saved by AUTO
 */
@TeleOp(name = "Biobuzz TeleOp", group = "Biobuzz")
public class BiobuzzTeleOp extends OpMode {
    private static final double RPM_TRIM_STEP = 50.0;
    private static final long RELOCALIZE_HOLD_MS = 1000;
    private static final long TELEMETRY_PERIOD_MS = 100;

    private Hubs hubs;
    private Follower follower;
    private Claw claw;
    private Shooter shooter;
    private Turret turret;

    private Alliance alliance;
    private boolean useSavedPose;
    private boolean firing = false;
    private double rpmTrim = 0.0;
    private long relocalizeHeldSince = 0;
    private boolean wasReady = false;
    private long lastLoopNs = 0;
    private double loopMs = 0.0;
    private long lastTelemetryMs = 0;

    @Override
    public void init() {
        hubs = new Hubs(hardwareMap);
        Battery battery = new Battery(hardwareMap);
        follower = Constants.create(hardwareMap);
        claw = new Claw(hardwareMap);
        shooter = new Shooter(hardwareMap, battery);
        hubs.clearCache();
        turret = new Turret(hardwareMap, battery);

        alliance = RobotState.alliance;
        useSavedPose = RobotState.pose != null;
        applyStartState();
    }

    private void applyStartState() {
        turret.setAlliance(alliance);
        if (useSavedPose) {
            follower.setPose(RobotState.pose);
            turret.setAngleReference(RobotState.turretAngleDeg);
            turret.setUpCell(RobotState.upCell);
        } else {
            follower.setPose(Field.startPose(alliance));
            // no hand-off: the turret has to be facing forward right now
            turret.setAngleReference(0.0);
            turret.setUpCell(Field.startingUpCell(alliance));
        }
    }

    @Override
    public void init_loop() {
        hubs.clearCache();

        boolean changed = false;
        if (gamepad1.xWasPressed()) {
            alliance = Alliance.BLUE;
            changed = true;
        }
        if (gamepad1.bWasPressed()) {
            alliance = Alliance.RED;
            changed = true;
        }
        if (gamepad1.aWasPressed() && RobotState.pose != null) {
            useSavedPose = !useSavedPose;
            changed = true;
        }
        if (changed) {
            applyStartState();
        }

        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("Alliance", "%s   (gamepad 1: X blue, B red)", alliance);
        if (useSavedPose) {
            telemetry.addData("Start", "position saved by AUTO (A to ignore)");
        } else if (RobotState.pose != null) {
            telemetry.addData("Start", "default start pose, turret facing forward (A to use AUTO's)");
        } else {
            telemetry.addData("Start", "default start pose, turret must face forward");
        }
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Up CELL", turret.getUpCell());
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setMode(Turret.Mode.AUTO_AIM);
        shooter.setEnabled(true);
        lastLoopNs = System.nanoTime();
    }

    @Override
    public void loop() {
        hubs.clearCache();

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        Pose pose = follower.pose();

        handleTurretControls();
        handleRelocalize();

        turret.update(pose, follower.velocity());
        shooter.setShotDistance(turret.getDistance(), rpmTrim);
        shooter.update();

        handleClaw();

        boolean ready = turret.isOnTarget() && shooter.atSpeed() && Field.isOnOpeningSide(pose, turret.getUpCell());
        if (ready && !wasReady) {
            gamepad1.rumble(150);
        }
        wasReady = ready;

        RobotState.save(alliance, pose, turret);
        addTelemetry(pose);
    }

    @Override
    public void stop() {
        claw.stop();
        claw.close();
        shooter.stop();
        turret.stop();
    }

    private void handleTurretControls() {
        // evaluate both pads every loop: xWasPressed() consumes the press
        boolean flipCell = gamepad1.yWasPressed();
        flipCell |= gamepad2.yWasPressed();
        if (flipCell) {
            turret.flipUpCell();
        }

        if (gamepad1.dpadRightWasPressed()) turret.adjustTrim(5);
        if (gamepad1.dpadLeftWasPressed()) turret.adjustTrim(-5);
        if (gamepad1.dpadUpWasPressed()) turret.adjustTrim(10);
        if (gamepad1.dpadDownWasPressed()) turret.adjustTrim(-10);
        if (gamepad1.backWasPressed()) turret.resetTrim();

        if (gamepad2.dpadRightWasPressed()) turret.adjustTrim(3);
        if (gamepad2.dpadLeftWasPressed()) turret.adjustTrim(-3);
        if (gamepad2.rightBumperWasPressed()) turret.adjustTrim(10);
        if (gamepad2.leftBumperWasPressed()) turret.adjustTrim(-10);
        if (gamepad2.dpadUpWasPressed()) rpmTrim += RPM_TRIM_STEP;
        if (gamepad2.dpadDownWasPressed()) rpmTrim -= RPM_TRIM_STEP;

        if (gamepad2.aWasPressed()) {
            turret.setMode(turret.getMode() == Turret.Mode.AUTO_AIM ? Turret.Mode.HOLD_FORWARD : Turret.Mode.AUTO_AIM);
        }
    }

    private void handleRelocalize() {
        if (!gamepad2.x) {
            relocalizeHeldSince = 0;
            return;
        }
        long now = System.currentTimeMillis();
        if (relocalizeHeldSince == 0) {
            relocalizeHeldSince = now;
        } else if (relocalizeHeldSince > 0 && now - relocalizeHeldSince >= RELOCALIZE_HOLD_MS) {
            follower.setPose(Field.startPose(alliance));
            gamepad2.rumble(300);
            // -1: already done, wait for release
            relocalizeHeldSince = -1;
        }
    }

    private void handleClaw() {
        if (gamepad1.right_bumper) {
            claw.feedForShot();
            claw.release();
            firing = true;
            return;
        }

        if (firing) {
            claw.close();
            claw.stop();
            firing = false;
        }

        if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            claw.close();
            claw.run();
        } else if (gamepad1.left_trigger > 0.5 || gamepad1.left_bumper) {
            claw.close();
            claw.stop();
        }
    }

    private void addTelemetry(Pose pose) {
        long nowNs = System.nanoTime();
        loopMs += 0.2 * ((nowNs - lastLoopNs) / 1e6 - loopMs);
        lastLoopNs = nowNs;

        // formatting telemetry every loop costs loop time; the DS only refreshes ~10 Hz anyway
        long nowMs = nowNs / 1_000_000L;
        if (nowMs - lastTelemetryMs < TELEMETRY_PERIOD_MS) {
            return;
        }
        lastTelemetryMs = nowMs;

        boolean openingSide = Field.isOnOpeningSide(pose, turret.getUpCell());
        telemetry.addData("Alliance", "%s %s", alliance, openingSide ? "" : "  WRONG SIDE OF HIVE");
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        FusedPinpointLocalizer localizer = Constants.fusedLocalizer(follower);
        if (localizer != null && localizer.usingImuFallback()) {
            telemetry.addData("Localizer", "PINPOINT %s - heading from hub IMU, position frozen", localizer.status());
        }
        turret.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.addData("RPM trim", "%+.0f", rpmTrim);
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.update();
    }
}
