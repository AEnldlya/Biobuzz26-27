package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Ballistics;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.GamePiece;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Everything the robot code can touch, simulated: a HardwareMap full of fake devices, the
 * physics behind them, the BIOBUZZ field (POLLEN and NECTAR on the tiles, a bi-stable HIVE
 * that tips), balls in the robot, and what happens to a ball that gets shot.
 *
 * Ball handling follows the intake rules: the intake and transfer motors running with the
 * blocker engaged collects pieces in front of the intake (possession limit 4); the same motors
 * with the blocker released feed one ball into the flywheel every shotIntervalS. A shot leaves
 * at the exit speed the flywheel RPM gives (Ballistics: wheel surface speed times the
 * compression-dependent transfer ratio), at the hood's exit angle, in the direction the turret
 * is really pointing, plus the robot's own velocity, from LAUNCH_HEIGHT_IN at the turret
 * pivot; it then flies under gravity and air drag. It scores when it crosses the up CELL's
 * tilted 20 x 14 in mouth (Field.cellMouth) going in.
 *
 * The HIVE starts with 3 NECTAR in its up CELL and tips once the CELL holds
 * Field.TIP_POLLEN_EQUIVALENTS: HIVE_TIP_TIME_S later the other CELL is up (20 points) and the
 * contents are dumped on the tiles on the side that was up. Shots during the swing are lost.
 *
 * Constants.localizerFactory / drivetrainFactory and PollenVision.CAMERA_FACTORY are pointed
 * at the models, so the real OpModes run unmodified.
 */
public class SimWorld {
    public static class Piece {
        public double x, y;
        public GamePiece type;
        public boolean collected;
        public double collectedAtS = Double.NaN;

        Piece(double x, double y, GamePiece type) {
            this.x = x;
            this.y = y;
            this.type = type;
        }
    }

    public static class Shot {
        public double timeS;
        public double rpm;
        public double exitSpeedInS;
        public double exitAngleDeg;
        public double turretDeg;
        public double trueAimDeg;
        public double distanceIn;
        public Field.CellSide cell;
        /** where the ball crossed the mouth plane (or came closest to the mouth centre) */
        public double landX, landY, landZ;
        public double missIn;
        public boolean scored;
        public boolean dribbled;
        public boolean duringTip;
    }

    public final Alliance alliance;
    public final HardwareMap hardwareMap;
    public final SimRobot robot;
    public final SimTurret turret;
    public final SimShooter shooter;
    public final SimCamera camera;
    public final SimTelemetry telemetry = new SimTelemetry();
    public final SimReport report;
    public final List<Piece> pieces = new ArrayList<>();
    public final List<Shot> shots = new ArrayList<>();

    public final SimDevices.SimMotor intakeMotor = new SimDevices.SimMotor("intakeMotor");
    public final SimDevices.SimMotor transferMotor = new SimDevices.SimMotor("transferMotor");
    public final SimDevices.SimServo blocker = new SimDevices.SimServo("blockServo");
    public final SimDevices.SimServo hood = new SimDevices.SimServo("aimServo");
    public final SimDevices.SimCRServo turretServo1 = new SimDevices.SimCRServo("turretServo");
    public final SimDevices.SimCRServo turretServo2 = new SimDevices.SimCRServo("turretServo2");
    public final SimDevices.SimMotor shooterMotor1 = new SimDevices.SimMotor("shooterMotor");
    public final SimDevices.SimMotor shooterMotor2 = new SimDevices.SimMotor("shooterMotor2");
    public final SimDevices.SimVoltageSensor battery = new SimDevices.SimVoltageSensor();

    // HIVE state
    public Field.CellSide upCell;
    public final List<GamePiece> inUpCell = new ArrayList<>();
    public double hiveLoadEq = 0.0;
    public int tips = 0;
    public boolean tipping = false;
    private double tipStartS = 0;

    // robot / scoring state
    public int ballsInRobot = 0;
    public int ballsCollected = 0;
    public int ballsScored = 0;
    public int maxBalls = Field.POSSESSION_LIMIT;
    public boolean left = false;
    /** the intake mouth is at the front edge of the 12 in robot */
    public double intakeMouthForwardIn = Field.ROBOT_HALF_IN + 1.0;
    public double intakeReachIn = 8.0;
    public double intakeHalfAngleDeg = 50.0;
    public double shotIntervalS = 0.3;
    public double minShotRpm = 900.0;

    private double lastShotS = -10;
    private String label = "";
    private double lastRecordS = -1;
    private final Random random = new Random(5);

    public SimWorld(Alliance alliance, double startTurretDeg) {
        this.alliance = alliance;
        this.upCell = Field.startingUpCell(alliance);
        for (int i = 0; i < Field.NECTAR_STAGED_IN_UP_CELL; i++) {
            inUpCell.add(GamePiece.nectarOf(alliance));
            hiveLoadEq += GamePiece.nectarOf(alliance).pollenEquivalents();
        }
        robot = new SimRobot();
        turret = new SimTurret(turretServo1, turretServo2, startTurretDeg);
        shooter = new SimShooter(shooterMotor1, shooterMotor2, battery);
        camera = new SimCamera(this);
        report = new SimReport(this);

        hardwareMap = new SimHardwareMap();
        SimDevices.SimAnalogController analog = new SimDevices.SimAnalogController(turret::wire1Volts, turret::wire2Volts);
        hardwareMap.put(Turret.SERVO_NAME, turretServo1);
        hardwareMap.put(Turret.SERVO2_NAME, turretServo2);
        hardwareMap.put(Turret.FEEDBACK_NAME, new AnalogInput(analog, 0));
        hardwareMap.put(Turret.FEEDBACK2_NAME, new AnalogInput(analog, 1));
        hardwareMap.put(Intake.INTAKE_MOTOR_NAME, intakeMotor);
        hardwareMap.put(Intake.TRANSFER_MOTOR_NAME, transferMotor);
        hardwareMap.put(Intake.BLOCKER_SERVO_NAME, blocker);
        hardwareMap.put("aimServo", hood);
        hardwareMap.put("shooterMotor", shooterMotor1);
        hardwareMap.put("shooterMotor2", shooterMotor2);
        hardwareMap.put("battery", battery);
        hardwareMap.voltageSensor.put("battery", battery);
        for (String name : new String[] {"frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor"}) {
            hardwareMap.put(name, new SimDevices.SimMotor(name));
        }

        robot.setOnStep(this::stepRest);
        Constants.localizerFactory = hw -> robot;
        Constants.drivetrainFactory = hw -> robot;
        PollenVision.CAMERA_FACTORY = hw -> camera;
    }

    public Piece addPiece(GamePiece type, double x, double y) {
        Piece p = new Piece(x, y, type);
        pieces.add(p);
        return p;
    }

    /** Stage the tiles as the rules do: 4 POLLEN in each GARDEN. (FLOWER POLLEN sit inside the FLOWERS.) */
    public SimWorld stagePerRules() {
        for (double[] p : Field.RED_GARDEN_POLLEN) {
            addPiece(GamePiece.POLLEN, p[0], p[1]);
            Pose m = Alliance.BLUE.fromRed(new Pose(p[0], p[1]));
            addPiece(GamePiece.POLLEN, m.x(), m.y());
        }
        return this;
    }

    /** a caption for the replay (e.g. the auto state) */
    public void setLabel(String label) {
        this.label = label;
    }

    public String label() {
        return label;
    }

    /** Everything except the drivetrain, after each robot physics step. */
    private void stepRest() {
        double dt = robot.lastDt();
        turret.step(dt);
        shooter.step(dt);
        camera.step();
        handleBalls();
        if (!left && Field.hasLeft(alliance, robot.truePose())) {
            left = true;
        }
        double t = robot.simTimeS();
        if (tipping && t - tipStartS >= Field.HIVE_TIP_TIME_S) {
            completeTip();
        }
        if (t - lastRecordS >= 0.05) {
            lastRecordS = t;
            report.record();
        }
    }

    private void completeTip() {
        tipping = false;
        tips++;
        // the contents land on the tiles on the side that was up
        Pose mouth = Field.cellAimPoint(alliance, upCell);
        double out = upCell == Field.CellSide.FAR ? 1.0 : -1.0;
        for (GamePiece type : inUpCell) {
            double x = mouth.x() + (random.nextDouble() - 0.5) * 20.0;
            double y = mouth.y() + out * (14.0 + random.nextDouble() * 20.0);
            addPiece(type, x, y);
        }
        inUpCell.clear();
        hiveLoadEq = 0.0;
        upCell = upCell.flipped();
    }

    private boolean intakeRunning() {
        return intakeMotor.effectivePower() > 0.3 && transferMotor.effectivePower() > 0.3;
    }

    public boolean blockerEngaged() {
        return Math.abs(blocker.getPosition() - Intake.BLOCKER_ENGAGED) < 0.02;
    }

    private void handleBalls() {
        if (!intakeRunning()) {
            return;
        }
        Pose pose = robot.truePose();
        if (blockerEngaged()) {
            if (ballsInRobot >= maxBalls) {
                return;
            }
            double mouthX = pose.x() + intakeMouthForwardIn * Math.cos(pose.heading());
            double mouthY = pose.y() + intakeMouthForwardIn * Math.sin(pose.heading());
            for (Piece piece : pieces) {
                if (piece.collected || !piece.type.controllableBy(alliance)) {
                    continue;
                }
                double dx = piece.x - mouthX;
                double dy = piece.y - mouthY;
                if (Math.hypot(dx, dy) > intakeReachIn) {
                    continue;
                }
                double rel = Angle.normalizeSigned(Math.atan2(piece.y - pose.y(), piece.x - pose.x()) - pose.heading());
                if (Math.abs(Math.toDegrees(rel)) > intakeHalfAngleDeg) {
                    continue;
                }
                piece.collected = true;
                piece.collectedAtS = robot.simTimeS();
                ballsInRobot++;
                ballsCollected++;
                if (ballsInRobot >= maxBalls) {
                    break;
                }
            }
        } else if (ballsInRobot > 0 && robot.simTimeS() - lastShotS >= shotIntervalS) {
            lastShotS = robot.simTimeS();
            fire(pose);
        }
    }

    private void fire(Pose pose) {
        Shot shot = new Shot();
        shot.timeS = robot.simTimeS();
        shot.rpm = shooter.rpm();
        shot.turretDeg = turret.ringDeg();
        shot.trueAimDeg = trueAimDeg();
        shot.cell = upCell;
        shot.duringTip = tipping;
        ballsInRobot--;
        shooter.ballThrough();

        double heading = pose.heading();
        double pivotX = pose.x() + Turret.PIVOT_FORWARD_IN * Math.cos(heading) - Turret.PIVOT_LEFT_IN * Math.sin(heading);
        double pivotY = pose.y() + Turret.PIVOT_FORWARD_IN * Math.sin(heading) + Turret.PIVOT_LEFT_IN * Math.cos(heading);
        Field.CellMouth mouth = Field.cellMouth(alliance, upCell);
        shot.distanceIn = Math.hypot(mouth.cx - pivotX, mouth.cy - pivotY);
        shot.exitAngleDeg = Ballistics.hoodAngleForServo(hood.getPosition());
        if (shot.rpm < minShotRpm) {
            shot.dribbled = true;
        }
        shot.exitSpeedInS = shot.dribbled ? 30.0 : Ballistics.exitSpeedInS(shot.rpm);

        Velocity v = robot.trueFieldVelocity();
        flyIntoCell(shot, pivotX, pivotY, heading - Math.toRadians(shot.turretDeg), v.vx, v.vy, mouth);
        if (shot.duringTip) {
            shot.scored = false; // launching at a swinging HIVE: lost
        }
        if (shot.scored) {
            ballsScored++;
            inUpCell.add(GamePiece.POLLEN);
            hiveLoadEq += GamePiece.POLLEN.pollenEquivalents();
            if (!tipping && hiveLoadEq >= Field.TIP_POLLEN_EQUIVALENTS) {
                tipping = true;
                tipStartS = robot.simTimeS();
            }
        }
        shots.add(shot);
    }

    /**
     * Fly the ball in 3D: the 2D drag trajectory along the launch bearing, plus the robot's
     * velocity carried along, and test each step against the tilted mouth plane.
     */
    void flyIntoCell(Shot shot, double x0, double y0, double bearing, double vx, double vy, Field.CellMouth mouth) {
        Ballistics.Flight f = Ballistics.fly(shot.exitSpeedInS, shot.exitAngleDeg, Ballistics.LAUNCH_HEIGHT_IN, 400.0);
        double ballRadius = Ballistics.BALL_DIAMETER_IN / 2.0;
        double cosB = Math.cos(bearing), sinB = Math.sin(bearing);
        double best = Double.MAX_VALUE;
        double prevSide = Double.NaN, px = 0, py = 0, pz = 0;
        shot.scored = false;
        for (int i = 0; i < f.n; i++) {
            double x = x0 + f.x[i] * cosB + vx * f.t[i];
            double y = y0 + f.x[i] * sinB + vy * f.t[i];
            double z = Ballistics.LAUNCH_HEIGHT_IN + f.z[i];
            double side = mouth.side(x, y, z);
            double dist = Math.sqrt((x - mouth.cx) * (x - mouth.cx) + (y - mouth.cy) * (y - mouth.cy) + (z - mouth.cz) * (z - mouth.cz));
            if (dist < best) {
                best = dist;
                shot.landX = x;
                shot.landY = y;
                shot.landZ = z;
                shot.missIn = dist;
            }
            if (i > 0 && prevSide > 0 && side <= 0) {
                // crossed the mouth plane going in: where?
                double fr = prevSide / (prevSide - side);
                double ix = px + fr * (x - px), iy = py + fr * (y - py), iz = pz + fr * (z - pz);
                shot.landX = ix;
                shot.landY = iy;
                shot.landZ = iz;
                shot.missIn = Math.sqrt((ix - mouth.cx) * (ix - mouth.cx) + (iy - mouth.cy) * (iy - mouth.cy) + (iz - mouth.cz) * (iz - mouth.cz));
                shot.scored = !shot.dribbled && mouth.inside(ix, iy, iz, ballRadius);
                return;
            }
            prevSide = side;
            px = x;
            py = y;
            pz = z;
        }
    }

    /** Tests: fire one ball right now from wherever the robot is, with the wheel at rpm. */
    public Shot testShot(double rpm) {
        shooter.setRpm(rpm);
        int before = shots.size();
        int balls = ballsInRobot;
        ballsInRobot = Math.max(1, balls);
        fire(robot.truePose());
        ballsInRobot = balls;
        return shots.get(before);
    }

    /** The exact turret angle (deg, positive right) that points at the up CELL mouth from the true pose. */
    public double trueAimDeg() {
        Pose pose = robot.truePose();
        double heading = pose.heading();
        double pivotX = pose.x() + Turret.PIVOT_FORWARD_IN * Math.cos(heading) - Turret.PIVOT_LEFT_IN * Math.sin(heading);
        double pivotY = pose.y() + Turret.PIVOT_FORWARD_IN * Math.sin(heading) + Turret.PIVOT_LEFT_IN * Math.cos(heading);
        Pose cell = Field.cellAimPoint(alliance, upCell);
        double bearing = Math.atan2(cell.y() - pivotY, cell.x() - pivotX);
        return -Math.toDegrees(Angle.normalizeSigned(bearing - heading));
    }

    /** turret pointing error right now, deg (ignores shoot-on-the-move lead) */
    public double turretErrorDeg() {
        return Math.toDegrees(Angle.normalizeSigned(Math.toRadians(turret.ringDeg() - trueAimDeg())));
    }

    public int shotsFired() {
        return shots.size();
    }

    public int collectedCount() {
        int n = 0;
        for (Piece p : pieces) {
            if (p.collected) {
                n++;
            }
        }
        return n;
    }

    public boolean parked() {
        return Field.isParked(alliance, robot.truePose());
    }

    /** AUTO points as the rules count them right now: LEAVE 3, TIP 20 each, PARK 5, 2 per element left in the up CELL */
    public int autoPoints() {
        return (left ? 3 : 0) + tips * 20 + (parked() ? 5 : 0) + 2 * inUpCell.size();
    }

    public String pointsBreakdown() {
        return String.format("LEAVE %d + TIP %d + PARK %d + in CELL %d = %d",
                left ? 3 : 0, tips * 20, parked() ? 5 : 0, 2 * inUpCell.size(), autoPoints());
    }
}
