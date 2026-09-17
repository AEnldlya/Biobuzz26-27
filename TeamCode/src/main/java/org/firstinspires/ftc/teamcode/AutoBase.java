package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FusedPinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.PathProfiles;
import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

/**
 * The whole AUTO, shared by RedAuto and BlueAuto (they only pick the alliance).
 *
 *   TO_SCORE  drive from the start to the scoring spot, turret already tracking the CELL
 *   SHOOT     fire the preload once the turret is on target and the flywheel is at speed
 *   TO_SCAN   drive to the pollen zone
 *   SCAN      hold still (optionally sweeping the heading) while the Limelight maps pollen
 *   APPROACH  drive to just short of the biggest pile of TARGET_POLLEN, intake facing it
 *   COLLECT   intake on, push through the pile
 *   RETURN    back to the scoring spot, then SHOOT again; repeat up to MAX_CYCLES
 *   PARK      when done or when the clock runs low
 *
 * Every waypoint is written for RED in Field and mirrored for BLUE. The turret and shooter
 * update every loop no matter the state, so the turret is always tracking the goal and the
 * flywheel is always at the right speed for the current distance.
 */
public abstract class AutoBase extends OpMode {
    public static int PRELOAD_BALLS = 3;
    public static int CYCLE_BALLS = 3;
    public static int MAX_CYCLES = 2;
    public static Pollen TARGET_POLLEN = Pollen.PURPLE;

    /** seconds the blocker is open per ball, then closed between balls for flywheel recovery */
    public static double SHOT_FEED_S = 0.35;
    public static double BETWEEN_SHOTS_S = 0.25;
    /** fire anyway after waiting this long for turret / flywheel ready (don't waste the auto) */
    public static double AIM_TIMEOUT_S = 2.5;

    public static double SCAN_TIME_S = 1.0;
    /** heading sweep while scanning (0 = stay still); each side is held for half the scan */
    public static double SCAN_SWEEP_DEG = 25.0;
    /** stop this far short of the pollen pile, then push PUSH_THROUGH_IN past its centre */
    public static double STANDOFF_IN = 14.0;
    public static double PUSH_THROUGH_IN = 8.0;
    /** 0 = intake on the robot's front, Math.PI = on its back */
    public static double INTAKE_HEADING_OFFSET = 0.0;
    public static double INTAKE_EXTRA_S = 0.6;
    public static double FALLBACK_INTAKE_S = 1.5;

    public static double AUTO_LENGTH_S = 30.0;
    /** start parking when this much time is left */
    public static double PARK_RESERVE_S = 4.0;
    /** don't start a new cycle unless this much time is left */
    public static double CYCLE_NEEDS_S = 11.0;

    protected enum State {
        TO_SCORE, SHOOT, TO_SCAN, SCAN, APPROACH, COLLECT, RETURN, PARK, DONE
    }

    private enum Volley {
        WAIT_READY, FEEDING, RECOVER, DONE
    }

    protected Hubs hubs;
    protected Follower follower;
    protected FusedPinpointLocalizer localizer;
    protected Claw claw;
    protected Shooter shooter;
    protected Turret turret;
    protected PollenVision vision;

    private final Timer stateTimer = new Timer();
    private final Timer volleyTimer = new Timer();
    private final Timer opmodeTimer = new Timer();

    private State state = State.TO_SCORE;
    private boolean entering = true;
    private long collectPathDoneNs = 0;
    private Volley volley = Volley.DONE;
    private int shotsLeft = 0;
    private int shotsFired = 0;
    private int cycles = 0;
    private boolean sweptRight = false;
    private PollenVision.Cluster targetCluster = null;
    private Pose approachPose = null;
    private boolean fallbackPickup = false;
    private String note = "";

    private Pose startPose;
    private Pose scorePose;
    private Pose scanPose;
    private Pose fallbackPickupPose;
    private Pose parkPose;

    protected abstract Alliance alliance();

    @Override
    public void init() {
        Alliance alliance = alliance();
        startPose = Field.startPose(alliance);
        scorePose = alliance.fromRed(Field.RED_SCORE);
        scanPose = alliance.fromRed(Field.RED_SCAN);
        fallbackPickupPose = alliance.fromRed(Field.RED_PICKUP_FALLBACK);
        parkPose = alliance.fromRed(Field.RED_PARK);

        hubs = new Hubs(hardwareMap);
        Battery battery = new Battery(hardwareMap);
        follower = Constants.create(hardwareMap);
        localizer = Constants.fusedLocalizer(follower);
        follower.setPose(startPose);
        claw = new Claw(hardwareMap);
        shooter = new Shooter(hardwareMap, battery);
        hubs.clearCache();
        // the turret must be facing forward at init
        turret = new Turret(hardwareMap);
        turret.setAlliance(alliance);
        turret.setUpCell(Field.startingUpCell(alliance));
        vision = new PollenVision(hardwareMap, localizer != null ? localizer::poseAt : nano -> follower.pose());
        vision.setTarget(TARGET_POLLEN);

        claw.close();
        RobotState.save(alliance, startPose, turret);
    }

    @Override
    public void init_loop() {
        hubs.clearCache();
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        if (localizer != null) {
            telemetry.addData("Pinpoint", "%s   hub IMU %s", localizer.status(), localizer.hasHubImu() ? "ok" : "none");
        }
        telemetry.addData("Limelight", vision.isConnected() ? "found, looking for " + TARGET_POLLEN : "NOT FOUND (fallback pickup only)");
        telemetry.addData("Turret", "must be facing forward now");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        turret.setMode(Turret.Mode.AUTO_AIM);
        shooter.setEnabled(true);
        vision.start();
        setState(State.TO_SCORE);
    }

    @Override
    public void loop() {
        hubs.clearCache();
        follower.update();
        Pose pose = follower.pose();

        // the turret tracks the CELL every loop, whatever the drivetrain is doing
        turret.update(pose, follower.velocity());
        shooter.setShotDistance(turret.getDistance(), 0.0);
        shooter.update();
        vision.update();

        runStateMachine(pose);
        RobotState.save(alliance(), pose, turret);
        addTelemetry(pose);
    }

    @Override
    public void stop() {
        vision.stop();
        claw.stop();
        claw.close();
        shooter.stop();
        turret.stop();
    }

    // ------------------------------------------------------------------ state machine

    private void runStateMachine(Pose pose) {
        double left = AUTO_LENGTH_S - opmodeTimer.seconds();
        if (left <= PARK_RESERVE_S && state != State.PARK && state != State.DONE) {
            note = "clock: parking";
            abortVolley();
            setState(State.PARK);
        }

        boolean enter = entering;
        entering = false;

        switch (state) {
            case TO_SCORE:
                if (enter) {
                    followTo(scorePose, PathProfiles.score());
                }
                if (pathDone()) {
                    startVolley(PRELOAD_BALLS);
                    setState(State.SHOOT);
                }
                break;

            case SHOOT:
                if (runVolley(pose)) {
                    boolean canCycle = cycles < MAX_CYCLES && left >= CYCLE_NEEDS_S;
                    if (canCycle) {
                        setState(State.TO_SCAN);
                    } else {
                        setState(State.PARK);
                    }
                }
                break;

            case TO_SCAN:
                if (enter) {
                    followTo(scanPose, PathProfiles.transit());
                }
                if (pathDone()) {
                    setState(State.SCAN);
                }
                break;

            case SCAN:
                scan(enter);
                if (stateTimer.seconds() >= SCAN_TIME_S) {
                    pickTarget(pose);
                }
                break;

            case APPROACH:
                if (pathDone()) {
                    claw.close();
                    claw.run();
                    if (fallbackPickup) {
                        // no camera target: sit on the fallback spot and intake
                        follower.hold(fallbackPickupPose);
                    } else {
                        Pose through = PollenVision.throughPose(approachPose, targetCluster, PUSH_THROUGH_IN, INTAKE_HEADING_OFFSET);
                        follower.follow(line(pose, through).constant(approachPose.heading()).with(PathProfiles.pickup()));
                    }
                    collectPathDoneNs = 0;
                    setState(State.COLLECT);
                }
                break;

            case COLLECT:
                if (fallbackPickup) {
                    if (stateTimer.seconds() >= FALLBACK_INTAKE_S) {
                        finishCollect();
                    }
                } else if (pathDone()) {
                    // keep the intake running a little after the push-through ends
                    long now = System.nanoTime();
                    if (collectPathDoneNs == 0) {
                        collectPathDoneNs = now;
                    } else if (now - collectPathDoneNs >= INTAKE_EXTRA_S * 1e9) {
                        finishCollect();
                    }
                }
                break;

            case RETURN:
                if (stateTimer.seconds() > 0.4) {
                    // balls are in: stop the intake so nothing is fed early
                    claw.stop();
                }
                if (pathDone()) {
                    cycles++;
                    startVolley(CYCLE_BALLS);
                    setState(State.SHOOT);
                }
                break;

            case PARK:
                if (enter) {
                    claw.stop();
                    claw.close();
                    followTo(parkPose, PathProfiles.park());
                }
                if (pathDone()) {
                    setState(State.DONE);
                }
                break;

            case DONE:
                break;
        }
    }

    private void finishCollect() {
        vision.clear(); // that pile is (hopefully) gone
        followTo(scorePose, PathProfiles.score());
        setState(State.RETURN);
    }

    /** Hold the scan pose, turned left for the first half of the scan and right for the second. */
    private void scan(boolean enter) {
        if (SCAN_SWEEP_DEG <= 0) {
            return;
        }
        boolean secondHalf = stateTimer.seconds() >= SCAN_TIME_S / 2.0;
        if (enter || secondHalf != sweptRight) {
            sweptRight = secondHalf;
            double sweep = Math.toRadians(SCAN_SWEEP_DEG) * (secondHalf ? -1 : 1);
            follower.hold(scanPose.withHeading(Angle.normalize(scanPose.heading() + sweep)));
        }
    }

    private void pickTarget(Pose pose) {
        targetCluster = vision.bestNear(scanPose, Field.MAX_POLLEN_CHASE_IN);
        if (targetCluster != null) {
            fallbackPickup = false;
            approachPose = PollenVision.approachPose(pose, targetCluster, STANDOFF_IN, INTAKE_HEADING_OFFSET);
            note = String.format("pollen at %.0f, %.0f (w %.2f)", targetCluster.x, targetCluster.y, targetCluster.weight);
            followTo(approachPose, PathProfiles.transit());
        } else {
            fallbackPickup = true;
            approachPose = fallbackPickupPose;
            note = "no pollen seen: fallback pickup";
            followTo(fallbackPickupPose, PathProfiles.pickup());
        }
        setState(State.APPROACH);
    }

    // ------------------------------------------------------------------ shooting

    private void startVolley(int balls) {
        shotsLeft = balls;
        volley = Volley.WAIT_READY;
        volleyTimer.reset();
    }

    private void abortVolley() {
        volley = Volley.DONE;
        claw.close();
        claw.stop();
    }

    protected boolean readyToFire(Pose pose) {
        return turret.isOnTarget() && shooter.atSpeed() && !follower.following();
    }

    /** Runs the current volley; true once every ball has been fed. */
    private boolean runVolley(Pose pose) {
        switch (volley) {
            case WAIT_READY:
                if (shotsLeft <= 0) {
                    volley = Volley.DONE;
                    return true;
                }
                if (!Field.isOnOpeningSide(pose, turret.getUpCell())) {
                    // shooting into the outside of the CELL is a G417 violation: don't
                    note = "wrong side of HIVE, not firing";
                    volley = Volley.DONE;
                    claw.close();
                    claw.stop();
                    return true;
                }
                if (readyToFire(pose) || volleyTimer.seconds() > AIM_TIMEOUT_S) {
                    claw.feedForShot();
                    claw.release();
                    volley = Volley.FEEDING;
                    volleyTimer.reset();
                }
                return false;

            case FEEDING:
                if (volleyTimer.seconds() >= SHOT_FEED_S) {
                    claw.close();
                    claw.stop();
                    shotsLeft--;
                    shotsFired++;
                    volley = Volley.RECOVER;
                    volleyTimer.reset();
                }
                return false;

            case RECOVER:
                if (volleyTimer.seconds() >= BETWEEN_SHOTS_S) {
                    volley = Volley.WAIT_READY;
                    volleyTimer.reset();
                }
                return false;

            default:
                return true;
        }
    }

    // ------------------------------------------------------------------ helpers

    /** Straight line from wherever the robot is to target, heading blending to target's. */
    protected void followTo(Pose target, Modifier[] profile) {
        Pose from = follower.pose();
        Path path = line(from, target).linear(from.heading(), target.heading()).with(profile);
        follower.follow(path);
    }

    /** true once the follower has finished its path (Pedro then holds the end pose). */
    protected boolean pathDone() {
        return !follower.following();
    }

    protected void setState(State next) {
        state = next;
        entering = true;
        stateTimer.reset();
    }

    protected State getState() {
        return state;
    }

    private void addTelemetry(Pose pose) {
        telemetry.addData("Auto", "%s  %.1fs  cycle %d  shots %d  %s", state, opmodeTimer.seconds(), cycles, shotsFired, note);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        if (localizer != null && localizer.usingImuFallback()) {
            telemetry.addData("Localizer", "PINPOINT %s - heading from hub IMU", localizer.status());
        }
        turret.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        vision.addTelemetry(telemetry);
        telemetry.update();
    }
}
