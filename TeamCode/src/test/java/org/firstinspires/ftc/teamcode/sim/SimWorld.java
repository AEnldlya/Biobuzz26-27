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
import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

import java.util.ArrayList;
import java.util.List;

/**
 * Everything the robot code can touch, simulated: a HardwareMap full of fake devices, the
 * physics behind them, pollen on the field, balls in the robot, and what happens to a ball
 * that gets shot.
 *
 * Ball handling follows the intake rules: the intake and transfer motors running with the
 * blocker engaged collects pollen that is in front of the intake; the same motors running
 * with the blocker released feeds one ball into the flywheel every shotIntervalS. A shot
 * leaves at the exit speed the flywheel RPM gives (Ballistics: wheel surface speed times the
 * compression-dependent transfer ratio), at the hood's exit angle, in the direction the turret
 * is really pointing, plus the robot's own velocity, from LAUNCH_HEIGHT_IN at the turret
 * pivot; it then flies under gravity and air drag. It scores when it crosses the up-CELL's
 * tilted opening plane (Field.cellOpeningNormal) inside Field.CELL_OPENING_RADIUS_IN.
 *
 * Constants.localizerFactory / drivetrainFactory and PollenVision.CAMERA_FACTORY are pointed
 * at the models, so the real OpModes run unmodified.
 */
public class SimWorld {
    public static class PollenPiece {
        public double x, y;
        public Pollen color;
        public boolean collected;
        public double collectedAtS = Double.NaN;

        PollenPiece(double x, double y, Pollen color) {
            this.x = x;
            this.y = y;
            this.color = color;
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
        /** where the ball crossed the opening plane (or came closest to the opening centre) */
        public double landX, landY, landZ;
        public double missIn;
        public boolean scored;
        public boolean dribbled;
    }

    public final Alliance alliance;
    public final HardwareMap hardwareMap;
    public final SimRobot robot;
    public final SimTurret turret;
    public final SimShooter shooter;
    public final SimCamera camera;
    public final SimTelemetry telemetry = new SimTelemetry();
    public final SimReport report;
    public final List<PollenPiece> pollen = new ArrayList<>();
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

    public Field.CellSide upCell;
    public int ballsInRobot = 0;
    public int ballsCollected = 0;
    public int ballsScored = 0;
    public int maxBalls = 3;
    /** the intake mouth is at the front edge of the 12 in robot */
    public double intakeMouthForwardIn = Field.ROBOT_HALF_IN + 1.0;
    public double intakeReachIn = 8.0;
    public double intakeHalfAngleDeg = 50.0;
    public double shotIntervalS = 0.3;
    public double minShotRpm = 900.0;

    private double lastShotS = -10;
    private String label = "";
    private double lastRecordS = -1;

    public SimWorld(Alliance alliance, double startTurretDeg) {
        this.alliance = alliance;
        this.upCell = Field.startingUpCell(alliance);
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

    public PollenPiece addPollen(Pollen color, double x, double y) {
        PollenPiece p = new PollenPiece(x, y, color);
        pollen.add(p);
        return p;
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
        double t = robot.simTimeS();
        if (t - lastRecordS >= 0.05) {
            lastRecordS = t;
            report.record();
        }
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
            for (PollenPiece piece : pollen) {
                if (piece.collected) {
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
        ballsInRobot--;
        shooter.ballThrough();

        double heading = pose.heading();
        double pivotX = pose.x() + Turret.PIVOT_FORWARD_IN * Math.cos(heading) - Turret.PIVOT_LEFT_IN * Math.sin(heading);
        double pivotY = pose.y() + Turret.PIVOT_FORWARD_IN * Math.sin(heading) + Turret.PIVOT_LEFT_IN * Math.cos(heading);
        Pose cell = Field.cellAimPoint(alliance, upCell);
        shot.distanceIn = Math.hypot(cell.x() - pivotX, cell.y() - pivotY);
        shot.exitAngleDeg = Ballistics.hoodAngleForServo(hood.getPosition());
        if (shot.rpm < minShotRpm) {
            shot.dribbled = true;
        }
        shot.exitSpeedInS = shot.dribbled ? 30.0 : Ballistics.exitSpeedInS(shot.rpm);

        Velocity v = robot.trueFieldVelocity();
        flyIntoCell(shot, pivotX, pivotY, heading - Math.toRadians(shot.turretDeg), v.vx, v.vy, cell);
        if (shot.scored) {
            ballsScored++;
        }
        shots.add(shot);
    }

    /**
     * Fly the ball in 3D: the 2D drag trajectory along the launch bearing, plus the robot's
     * velocity carried along, and test each step against the tilted opening plane.
     */
    void flyIntoCell(Shot shot, double x0, double y0, double bearing, double vx, double vy, Pose cell) {
        Ballistics.Flight f = Ballistics.fly(shot.exitSpeedInS, shot.exitAngleDeg, Ballistics.LAUNCH_HEIGHT_IN, 400.0);
        double[] n = Field.cellOpeningNormal(upCell);
        double cx = cell.x(), cy = cell.y(), cz = Field.CELL_OPENING_HEIGHT_IN;
        double cosB = Math.cos(bearing), sinB = Math.sin(bearing);
        double best = Double.MAX_VALUE;
        double prevSide = Double.NaN, px = 0, py = 0, pz = 0;
        shot.scored = false;
        for (int i = 0; i < f.n; i++) {
            double x = x0 + f.x[i] * cosB + vx * f.t[i];
            double y = y0 + f.x[i] * sinB + vy * f.t[i];
            double z = Ballistics.LAUNCH_HEIGHT_IN + f.z[i];
            double side = n[0] * (x - cx) + n[1] * (y - cy) + n[2] * (z - cz);
            double dist = Math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy) + (z - cz) * (z - cz));
            if (dist < best) {
                best = dist;
                shot.landX = x;
                shot.landY = y;
                shot.landZ = z;
                shot.missIn = dist;
            }
            if (i > 0 && prevSide > 0 && side <= 0) {
                // crossed the opening plane going in: where?
                double fr = prevSide / (prevSide - side);
                double ix = px + fr * (x - px), iy = py + fr * (y - py), iz = pz + fr * (z - pz);
                double inPlane = Math.sqrt((ix - cx) * (ix - cx) + (iy - cy) * (iy - cy) + (iz - cz) * (iz - cz));
                shot.landX = ix;
                shot.landY = iy;
                shot.landZ = iz;
                shot.missIn = inPlane;
                shot.scored = !shot.dribbled && inPlane <= Field.CELL_OPENING_RADIUS_IN;
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

    /** The exact turret angle (deg, positive right) that points at the up-CELL from the true pose. */
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
        for (PollenPiece p : pollen) {
            if (p.collected) {
                n++;
            }
        }
        return n;
    }
}
