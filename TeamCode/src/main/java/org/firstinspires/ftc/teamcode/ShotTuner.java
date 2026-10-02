package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Builds ShotTable and tunes the PIDF gains live.
 *
 * Park at a distance and adjust RPM and hood until volleys go in. Then either copy the
 * "Table row" line into ShotTable the old way, or - better - read "EFFICIENCY_TRIM implied",
 * put that one number into Ballistics, and the generated table and its regression follow for
 * every distance. Values changed here are lost when the OpMode stops, so write them into the
 * code.
 *
 * Gamepad 1
 *   sticks: drive               hold right bumper: fire
 *   right trigger: intake       left trigger: stop intake
 *   dpad up/down: RPM +/-50     Y / A: RPM +/-250     dpad right/left: hood +/-0.01
 *   X: switch MANUAL values / TABLE values (manual starts from the table)
 *   B: HIVE tipped, aim at the other CELL
 *   hold BACK: flywheel at full power, read "kV estimate" once the speed settles
 * Gamepad 2
 *   dpad up/down: pick a gain   dpad right/left: gain x1.25 / /1.25   B: gain = 0
 *   A: turret auto-aim on/off (off = hold forward)
 *   hold BACK 1 s: ZERO THE TURRET here - point it forward first. Takes effect immediately and
 *     prints the two lines to paste into Turret.java so it survives a restart.
 * Init: gamepad 1 X = blue, B = red
 */
@TeleOp(name = "Shot Tuner", group = "Tuning")
public class ShotTuner extends OpMode {
    private static final long ZERO_HOLD_MS = 1000;
    private static final String[] GAIN_NAMES = {
            "Turret pos kP", "Turret pos kI", "Turret vel kP", "Turret vel kI", "Turret kV", "Turret kS",
            "Shooter kP", "Shooter kI", "Shooter kV", "Shooter kS"
    };
    private Hubs hubs;
    private Battery battery;
    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    private Alliance alliance;
    private boolean manualValues = true;
    private double manualRPM = Double.NaN;
    private double manualHood = Double.NaN;
    private int selectedGain = 0;
    private boolean firing = false;
    private long zeroHeldSince = 0;
    private String zeroNote = "";

    @Override
    public void init() {
        hubs = new Hubs(hardwareMap);
        battery = new Battery(hardwareMap);
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, battery);
        hubs.clearCache();
        turret = new Turret(hardwareMap);

        alliance = RobotState.alliance;
        applyStartState();
    }

    private void applyStartState() {
        turret.setAlliance(alliance);
        boolean saved = RobotState.pose != null;
        if (saved) {
            follower.setPose(RobotState.pose);
            turret.setUpCell(RobotState.upCell);
        } else {
            follower.setPose(Field.startPose(alliance));
            turret.setUpCell(Field.startingUpCell(alliance));
        }
        // same rule as TELEOP: with calibrated 1:1 wires the constructor already read the true
        // angle, and tuning against a turret that is lying about where it points is worse than
        // useless - the gains would be fitted to the wrong error
        if (!Turret.hasAbsoluteFeedback()) {
            turret.setAngleReference(saved ? RobotState.turretAngleDeg : 0.0);
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
        Pose pose = follower.pose();

        if (gamepad1.bWasPressed()) {
            turret.flipUpCell();
        }
        if (gamepad2.aWasPressed()) {
            turret.setMode(turret.getMode() == Turret.Mode.AUTO_AIM ? Turret.Mode.HOLD_FORWARD : Turret.Mode.AUTO_AIM);
        }
        turret.update(pose, follower.velocity());
        double distance = turret.getDistance();

        handleShotValues(distance);
        handleGains();
        handleTurretZero();

        shooter.setOverridePower(gamepad1.back ? 1.0 : Double.NaN);
        shooter.update();

        if (gamepad1.right_bumper) {
            intake.shoot();
            firing = true;
        } else {
            if (firing) {
                intake.stop();
                firing = false;
            }
            if (gamepad1.right_trigger > 0.5) {
                intake.intake();
            } else if (gamepad1.left_trigger > 0.5) {
                intake.stop();
            }
        }

        RobotState.save(alliance, pose, turret);
        addTelemetry(pose, distance);
    }

    @Override
    public void stop() {
        intake.stop();
        shooter.stop();
        turret.stop();
    }

    private void handleShotValues(double distance) {
        if (Double.isNaN(manualRPM)) {
            manualRPM = ShotTable.rpm(distance);
            manualHood = ShotTable.hood(distance);
        }
        if (gamepad1.xWasPressed()) {
            manualValues = !manualValues;
            if (manualValues) {
                manualRPM = ShotTable.rpm(distance);
                manualHood = ShotTable.hood(distance);
            }
        }
        if (gamepad1.dpadUpWasPressed()) manualRPM += 50;
        if (gamepad1.dpadDownWasPressed()) manualRPM -= 50;
        if (gamepad1.yWasPressed()) manualRPM += 250;
        if (gamepad1.aWasPressed()) manualRPM -= 250;
        if (gamepad1.dpadRightWasPressed()) manualHood += 0.01;
        if (gamepad1.dpadLeftWasPressed()) manualHood -= 0.01;
        manualRPM = Math.max(0.0, Math.min(7000.0, manualRPM));
        manualHood = Math.max(0.0, Math.min(1.0, manualHood));

        if (manualValues) {
            shooter.setTargetRPM(manualRPM);
            shooter.setHood(manualHood);
        } else {
            shooter.setShotDistance(distance, 0.0);
        }
    }

    /**
     * Zero the turret where it is standing: record both position wires' raw angles as "forward"
     * and tell the turret it is at 0. Held rather than tapped, because it redefines the
     * reference every later reading is measured against and a stray press mid-tuning would
     * quietly move the target by however far the turret happened to be pointing.
     *
     * This only lasts as long as the OpMode, so the two values are printed in a form that can
     * go straight into Turret.java.
     */
    private void handleTurretZero() {
        if (!gamepad2.back) {
            zeroHeldSince = 0;
            return;
        }
        long now = System.currentTimeMillis();
        if (zeroHeldSince == 0) {
            zeroHeldSince = now;
        } else if (zeroHeldSince > 0 && now - zeroHeldSince >= ZERO_HOLD_MS) {
            Turret.FORWARD_RAW_DEG = turret.getRawDeg();
            Turret.FORWARD_RAW2_DEG = turret.getRaw2Deg();
            turret.setAngleReference(0.0);
            zeroNote = String.format("FORWARD_RAW_DEG = %.1f;   FORWARD_RAW2_DEG = %.1f;",
                    Turret.FORWARD_RAW_DEG, Turret.FORWARD_RAW2_DEG);
            gamepad2.rumble(300);
            zeroHeldSince = -1; // done, wait for release
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
            case 0: return Turret.POS_kP;
            case 1: return Turret.POS_kI;
            case 2: return Turret.kP;
            case 3: return Turret.kI;
            case 4: return Turret.kV;
            case 5: return Turret.kS;
            case 6: return Shooter.kP;
            case 7: return Shooter.kI;
            case 8: return Shooter.kV;
            default: return Shooter.kS;
        }
    }

    private static void setGain(int index, double value) {
        switch (index) {
            case 0: Turret.POS_kP = value; break;
            case 1: Turret.POS_kI = value; break;
            case 2: Turret.kP = value; break;
            case 3: Turret.kI = value; break;
            case 4: Turret.kV = value; break;
            case 5: Turret.kS = value; break;
            case 6: Shooter.kP = value; break;
            case 7: Shooter.kI = value; break;
            case 8: Shooter.kV = value; break;
            default: Shooter.kS = value; break;
        }
    }

    /** starting value when bumping a gain up from zero */
    private static double defaultStep(int index) {
        switch (index) {
            case 0: return 1.0;      // turret pos kP, deg/s per deg
            case 1: return 0.1;      // turret pos kI
            case 2: return 0.0005;   // turret vel kP, power per deg/s
            case 3: return 0.0005;   // turret vel kI
            case 4: return 0.001;    // turret kV
            case 5: return 0.02;     // turret kS
            case 6: return 0.0001;   // shooter kP
            case 7: return 0.0001;   // shooter kI
            case 8: return 0.0001;   // shooter kV
            default: return 0.01;    // shooter kS
        }
    }

    private void addTelemetry(Pose pose, double distance) {
        telemetry.addData("Values", manualValues ? "MANUAL (X for table)" : "TABLE (X for manual)");
        telemetry.addData("Table row", "distance %.0f -> RPM %.0f, hood %.3f",
                distance, shooter.getTargetRPM(), shooter.getHood());
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        turret.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        if (gamepad1.back && shooter.getRPM() > 100) {
            double kVEstimate = (battery.voltage() / Battery.NOMINAL_VOLTAGE - Shooter.kS) / shooter.getRPM();
            telemetry.addData("kV estimate", "%.8f  (full power, %.2f V)", kVEstimate, battery.voltage());
        }
        // Ballistics is designed around one number to fit on the robot. Once a manual RPM
        // actually scores from this distance, that number can be read straight off here rather
        // than worked out by hand: exit speed is proportional to trim x RPM, so if the model
        // asks for rpmModel and reality needs manualRPM, the model's transfer is off by exactly
        // that ratio. Set EFFICIENCY_TRIM to this, run regenerate(), and the whole table follows.
        if (manualValues && manualRPM > 100 && !Double.isNaN(distance)) {
            double rpmModel = ShotTable.rpm(distance);
            telemetry.addData("EFFICIENCY_TRIM implied", "%.4f  (model %.0f rpm, you used %.0f at %.0f in)",
                    Ballistics.EFFICIENCY_TRIM * rpmModel / manualRPM, rpmModel, manualRPM, distance);
        }
        telemetry.addData("Gain (gamepad 2)", "%s = %.6f", GAIN_NAMES[selectedGain], getGain(selectedGain));
        if (zeroNote.isEmpty()) {
            telemetry.addData("Turret zero", "point it forward, then hold gamepad 2 BACK for 1 s");
        } else {
            telemetry.addData("TURRET ZEROED - paste into Turret.java", zeroNote);
        }
        telemetry.update();
    }
}
