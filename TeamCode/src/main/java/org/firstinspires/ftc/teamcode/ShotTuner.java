package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Collects ShotTable/regression measurements and tunes the PIDF gains live.
 *
 * Park at a distance, adjust RPM and hood until volleys go in, then copy the "Table row" line
 * into ShotTable. Cover the whole range you shoot from (at least 5 distances). Record the
 * physical hood angle above horizontal and measure release-to-target flight time separately.
 * Fit RPM, angle in degrees and flight time in seconds versus horizontal distance in inches;
 * enter coefficients and measured hood linkage mapping in ShotTable. Servo position is NOT
 * an angle. Values changed here are lost when the OpMode stops, so write them into the code.
 *
 * Gamepad 1
 *   sticks: drive               hold right bumper: fire
 *   right trigger: intake       left trigger: stop intake
 *   dpad up/down: RPM +/-50     Y / A: RPM +/-250     dpad right/left: hood +/-0.01
 *   X: switch MANUAL values / MODEL values (manual starts from the configured table/fits)
 *   B: HIVE tipped, aim at the other CELL
 *   hold BACK: flywheel at full power, read "kV estimate" once the speed settles
 * Gamepad 2
 *   dpad up/down: pick a gain   dpad right/left: gain x1.25 / /1.25   B: gain = 0
 *   A: turret auto-aim on/off (off = hold forward)
 *   hold X for 1 s: stationary at the measured start spot AND heading, reset field pose
 * Init: gamepad 1 X = blue, B = red
 */
@TeleOp(name = "Shot Tuner", group = "Tuning")
public class ShotTuner extends OpMode {
    private static final long RELOCALIZE_HOLD_MS = 1000;
    private static final String[] GAIN_NAMES = {
            "Turret kP", "Turret kI", "Turret kD", "Turret kV", "Turret kS",
            "Shooter kP", "Shooter kI", "Shooter kV", "Shooter kS"
    };
    private Hubs hubs;
    private Battery battery;
    private Follower follower;
    private FieldLocalizer localization;
    private Claw claw;
    private Shooter shooter;
    private Turret turret;

    private Alliance alliance;
    private boolean manualValues = true;
    private double manualRPM = Double.NaN;
    private double manualHood = Double.NaN;
    private int selectedGain = 0;
    private boolean firing = false;
    private long relocalizeHeldSince = 0;

    @Override
    public void init() {
        hubs = new Hubs(hardwareMap);
        battery = new Battery(hardwareMap);
        follower = Constants.create(hardwareMap);
        localization = (FieldLocalizer) follower.localizer;
        claw = new Claw(hardwareMap);
        shooter = new Shooter(hardwareMap, battery);
        hubs.clearCache();
        turret = new Turret(hardwareMap, battery);

        alliance = RobotState.alliance;
        applyStartState();
    }

    private void applyStartState() {
        turret.resetTracking();
        turret.setAlliance(alliance);
        if (RobotState.hasSavedPose()) {
            follower.setPose(RobotState.pose);
            turret.setAngleReference(RobotState.turretAngleDeg);
            turret.setUpCell(RobotState.upCell);
        } else {
            follower.setPose(Field.startPose(alliance));
            turret.setAngleReference(0.0);
            turret.setUpCell(Field.startingUpCell(alliance));
        }
    }

    @Override
    public void init_loop() {
        hubs.clearCache();
        if (gamepad1.xWasPressed()) {
            alliance = Alliance.BLUE;
            RobotState.clearPose();
            applyStartState();
        }
        if (gamepad1.bWasPressed()) {
            alliance = Alliance.RED;
            RobotState.clearPose();
            applyStartState();
        }
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("Alliance", "%s   (gamepad 1: X blue, B red)", alliance);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        localization.addTelemetry(telemetry);
        telemetry.addData("Reset pose", "g2 hold X 1 s, stationary at start spot AND heading");
        telemetry.addData("Shot calibration", ShotTable.calibrationStatus());
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setMode(Turret.Mode.AUTO_AIM);
        shooter.setEnabled(true);
    }

    @Override
    public void loop() {
        hubs.clearCache();

        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        handleRelocalize();
        Pose pose = follower.pose();

        if (gamepad1.bWasPressed()) {
            turret.flipUpCell();
        }
        if (gamepad2.aWasPressed()) {
            turret.setMode(turret.getMode() == Turret.Mode.AUTO_AIM ? Turret.Mode.HOLD_FORWARD : Turret.Mode.AUTO_AIM);
        }
        turret.update(localization.isReady() ? pose : null, follower.velocity());
        double distance = turret.getDistance();

        handleShotValues(distance);
        handleGains();

        shooter.setOverridePower(gamepad1.back ? 1.0 : Double.NaN);
        shooter.update();

        if (gamepad1.right_bumper) {
            claw.feedForShot();
            claw.release();
            firing = true;
        } else {
            if (firing) {
                claw.close();
                claw.stop();
                firing = false;
            }
            if (gamepad1.right_trigger > 0.5) {
                claw.close();
                claw.run();
            } else if (gamepad1.left_trigger > 0.5) {
                claw.stop();
            }
        }

        RobotState.save(alliance, localization.isReady() ? pose : null, turret);
        addTelemetry(pose, distance);
    }

    @Override
    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
        claw.stop();
        claw.close();
        shooter.stop();
        turret.stop();
    }

    private void handleShotValues(double distance) {
        if (Double.isNaN(manualRPM)) {
            seedManualValues(distance);
        }
        if (gamepad1.xWasPressed()) {
            manualValues = !manualValues;
            if (manualValues) {
                seedManualValues(distance);
            }
        }
        if (!Double.isFinite(manualRPM) || !Double.isFinite(manualHood)) {
            // A missing pose or unfinished fit must not turn into a guessed initial shot.
            shooter.setShotDistance(Double.NaN, 0.0);
            return;
        }
        if (gamepad1.dpadUpWasPressed()) manualRPM += 50;
        if (gamepad1.dpadDownWasPressed()) manualRPM -= 50;
        if (gamepad1.yWasPressed()) manualRPM += 250;
        if (gamepad1.aWasPressed()) manualRPM -= 250;
        if (gamepad1.dpadRightWasPressed()) manualHood += 0.01;
        if (gamepad1.dpadLeftWasPressed()) manualHood -= 0.01;
        manualRPM = Math.max(0.0, Math.min(Shooter.MAX_TARGET_RPM, manualRPM));
        manualHood = Math.max(0.0, Math.min(1.0, manualHood));

        if (manualValues) {
            shooter.setTargetRPM(manualRPM);
            shooter.setHood(manualHood);
        } else {
            shooter.setShotDistance(distance, 0.0);
        }
    }

    private void seedManualValues(double distance) {
        double candidateRPM = ShotTable.rpm(distance);
        double candidateHood = ShotTable.hood(distance);
        if (Double.isFinite(candidateRPM) && Double.isFinite(candidateHood)) {
            manualRPM = candidateRPM;
            manualHood = candidateHood;
        }
    }

    private void handleRelocalize() {
        if (!gamepad2.x) {
            relocalizeHeldSince = 0;
            return;
        }
        long now = System.nanoTime() / 1_000_000L;
        if (relocalizeHeldSince == 0) {
            relocalizeHeldSince = now;
        } else if (relocalizeHeldSince > 0 && now - relocalizeHeldSince >= RELOCALIZE_HOLD_MS) {
            follower.setPose(Field.startPose(alliance));
            turret.resetTracking();
            gamepad2.rumble(300);
            relocalizeHeldSince = -1;
        }
    }

    private void handleGains() {
        if (gamepad2.dpadUpWasPressed()) {
            selectedGain = (selectedGain + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
        }
        if (gamepad2.dpadDownWasPressed()) {
            selectedGain = (selectedGain + 1) % GAIN_NAMES.length;
        }
        if (gamepad2.dpadRightWasPressed()) {
            double value = getGain(selectedGain);
            setGain(selectedGain, value == 0.0 ? defaultStep(selectedGain) : value * 1.25);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            setGain(selectedGain, getGain(selectedGain) / 1.25);
        }
        if (gamepad2.bWasPressed()) {
            setGain(selectedGain, 0.0);
        }
    }

    private static double getGain(int index) {
        switch (index) {
            case 0: return Turret.kP;
            case 1: return Turret.kI;
            case 2: return Turret.kD;
            case 3: return Turret.kV;
            case 4: return Turret.kS;
            case 5: return Shooter.kP;
            case 6: return Shooter.kI;
            case 7: return Shooter.kV;
            default: return Shooter.kS;
        }
    }

    private static void setGain(int index, double value) {
        switch (index) {
            case 0: Turret.kP = value; break;
            case 1: Turret.kI = value; break;
            case 2: Turret.kD = value; break;
            case 3: Turret.kV = value; break;
            case 4: Turret.kS = value; break;
            case 5: Shooter.kP = value; break;
            case 6: Shooter.kI = value; break;
            case 7: Shooter.kV = value; break;
            default: Shooter.kS = value; break;
        }
    }

    /** starting value when bumping a gain up from zero */
    private static double defaultStep(int index) {
        switch (index) {
            case 0: return 0.001;
            case 1: return 0.0001;
            case 2: return 0.0001;
            case 3: return 0.001;
            case 4: return 0.02;
            case 5: return 0.0001;
            case 6: return 0.0001;
            case 7: return 0.0001;
            default: return 0.01;
        }
    }

    private void addTelemetry(Pose pose, double distance) {
        telemetry.addData("Values", manualValues ? "MANUAL (X for model)" : "MODEL (X for manual)");
        telemetry.addData("Table row", "distance %.2f in -> RPM %.0f, hood servo %.4f",
                distance, shooter.getTargetRPM(), shooter.getHood());
        double hoodAngleDeg = ShotTable.hoodPositionToAngleDegrees(shooter.getHood());
        if (Double.isFinite(hoodAngleDeg)) {
            telemetry.addData("Hood angle above horizontal", "%.2f deg (measured linkage map)", hoodAngleDeg);
        } else {
            telemetry.addData("Hood angle", "Measure degrees separately; servo linkage map is not calibrated");
        }
        telemetry.addData("Flight calibration", "Measure seconds from ball release to target; do not copy estimated flight time");
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        localization.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        if (gamepad1.back && shooter.getRPM() > 100) {
            double kVEstimate = (battery.voltage() / Battery.NOMINAL_VOLTAGE - Shooter.kS) / shooter.getRPM();
            telemetry.addData("kV estimate", "%.8f  (full power, %.2f V)", kVEstimate, battery.voltage());
        }
        telemetry.addData("Gain (gamepad 2)", "%s = %.6f", GAIN_NAMES[selectedGain], getGain(selectedGain));
        telemetry.update();
    }
}
