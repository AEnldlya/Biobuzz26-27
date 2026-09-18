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
import org.firstinspires.ftc.teamcode.vision.GamePiece;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

/**
 * The whole BIOBUZZ AUTO, shared by RedAuto and BlueAuto (they only pick the alliance).
 *
 * The plan follows the rules (Field): the robot starts on its alliance wall with 4 POLLEN, the
 * up CELL already holds 3 NECTAR, and 3 more POLLEN tip the HIVE (20 points).
 *
 *   TO_SCORE  drive to the shooting spot for the up CELL, turret already tracking it
 *   SHOOT     fire TIP_VOLLEY_BALLS (3) once the turret is on target and the flywheel at
 *             speed. The HIVE tips: the other CELL is now up and faces the other half of the
 *             field, so the turret is told to flip (flipUpCell) and the score pose changes.
 *   TO_SCAN   drive to the GARDEN corner (4 POLLEN in a line along the wall)
 *   SCAN      hold still (optionally sweeping the heading) while the Limelight maps POLLEN
 *   APPROACH  drive to just short of the biggest pile, intake facing it
 *   COLLECT   intake on, push through the pile (possession limit 4)
 *   RETURN    to the rear-side shooting spot, SHOOT everything into the new up CELL
 *             (2 points each if they stay in, and progress toward the next TIP)
 *   PARK      in the LOADING ZONE (5 points), also when the clock runs low
 *
 * LEAVE (3 points) happens by driving off the wall. Every waypoint is written for RED in
 * Field and mirrored for BLUE. The turret and shooter update every loop no matter the state.
 */
public abstract class AutoBase extends OpMode {
    public static int PRELOAD_BALLS = Field.PRELOAD_POLLEN;
    /** POLLEN to launch at the starting up CELL: with 3 NECTAR staged in it, 3 tip the HIVE */
    public static int TIP_VOLLEY_BALLS = 3;
    /** feeds attempted after a pickup (everything on board, up to the possession limit) */
    public static int CYCLE_BALLS = Field.POSSESSION_LIMIT;
    public static int MAX_CYCLES = 1;
    public static GamePiece TARGET_POLLEN = GamePiece.POLLEN;

    /**
     * Seconds the blocker is open per ball, then closed between balls for flywheel recovery.
     * The transfer feeds continuously while the blocker is open, so this window must be SHORT
     * enough that exactly one ball passes: with only 4 POLLEN of possession a wasted ball can
     * cost a TIP. Tune it on the robot (too brief and the servo never clears the ball).
     */
    public static double SHOT_FEED_S = 0.25;
    public static double BETWEEN_SHOTS_S = 0.30;
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
    public static double CYCLE_NEEDS_S = 12.0;

    /**
     * Pedro reports a path finished at its parametric end, which can be well before the
     * HEADING has settled (the translation converges within hundredths of an inch first). For
     * the shooting spots that does not matter, because the turret aims itself; for the scan
     * pose and the pickup approach it does, because the camera and the intake point where the
     * robot points. Those paths wait for the heading, up to PATH_SETTLE_TIMEOUT_S.
     */
    public static double PATH_HEADING_TOLERANCE_DEG = 6.0;
    public static double PATH_SETTLE_TIMEOUT_S = 1.2;

    public enum State {
        TO_SCORE, SHOOT, TO_SCAN, SCAN, APPROACH, COLLECT, RETURN, PARK, DONE
    }

    private enum Volley {
        WAIT_READY, FEEDING, RECOVER, DONE
    }

    protected Hubs hubs;
    protected Follower follower;
    protected FusedPinpointLocalizer localizer;
    protected Intake intake;
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
    private int tipsAssumed = 0;
    private boolean sweptRight = false;
    private PollenVision.Cluster targetCluster = null;
    private Pose approachPose = null;
    private Pose pathTarget = null;
    private boolean pathNeedsHeading = false;
    private long pathEndedNs = 0;
    private boolean fallbackPickup = false;
    private String note = "";

    private Pose startPose;
    private Pose scanPose;
    private Pose fallbackPickupPose;
    private Pose parkPose;

    protected abstract Alliance alliance();

    @Override
    public void init() {
        Alliance alliance = alliance();
        startPose = Field.startPose(alliance);
        scanPose = Field.scanPose(alliance);
        fallbackPickupPose = Field.fallbackPickupPose(alliance);
        parkPose = Field.parkPose(alliance);

        hubs = new Hubs(hardwareMap);
        Battery battery = new Battery(hardwareMap);
        follower = Constants.create(hardwareMap);
        localizer = Constants.fusedLocalizer(follower);
        follower.setPose(startPose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, battery);
        hubs.clearCache();
        // with calibrated 1:1 wires the turret reads its own angle; otherwise it must be
        // facing forward right now (init_loop says which)
        turret = new Turret(hardwareMap);
        turret.setAlliance(alliance);
        turret.setUpCell(Field.startingUpCell(alliance));
        vision = new PollenVision(hardwareMap, Constants.poseHistory(follower));
        vision.setTarget(TARGET_POLLEN);

        intake.stop();
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
        telemetry.addData("Turret", Turret.hasAbsoluteFeedback()
                ? "absolute from the position wires: it can be anywhere"
                : "must be FACING FORWARD now");
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
        intake.stop();
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
                    followTo(scorePose(), PathProfiles.score());
                }
                if (pathDone()) {
                    startVolley(Math.min(TIP_VOLLEY_BALLS, PRELOAD_BALLS));
                    setState(State.SHOOT);
                }
                break;

            case SHOOT:
                if (runVolley(pose)) {
                    if (tipsAssumed == 0 && shotsFired >= TIP_VOLLEY_BALLS) {
                        // 3 NECTAR were staged in that CELL; with our 3 POLLEN it tips. The
                        // other CELL is up now and faces the other half of the field.
                        tipsAssumed++;
                        turret.flipUpCell();
                        note = "HIVE tipped: " + turret.getUpCell() + " CELL up";
                    }
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
                    // the camera has to be looking at the GARDEN when SCAN starts
                    followTo(scanPose, PathProfiles.transit(), true);
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
                    intake.intake();
                    if (fallbackPickup) {
                        // no camera target: sit on the fallback spot and intake
                        follower.hold(fallbackPickupPose);
                        pathTarget = null;
                    } else {
                        Pose through = PollenVision.throughPose(approachPose, targetCluster, PUSH_THROUGH_IN, INTAKE_HEADING_OFFSET);
                        launch(line(pose, through).constant(approachPose.heading()).with(PathProfiles.pickup()), through, false);
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
                    intake.stop();
                }
                if (pathDone()) {
                    cycles++;
                    startVolley(CYCLE_BALLS);
                    setState(State.SHOOT);
                }
                break;

            case PARK:
                if (enter) {
                    intake.stop();
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
        followTo(scorePose(), PathProfiles.score());
        setState(State.RETURN);
    }

    /** shooting spot for whichever CELL is up right now */
    private Pose scorePose() {
        return Field.scorePose(alliance(), turret.getUpCell());
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
            // the intake has to be facing the pile before the push-through starts
            followTo(approachPose, PathProfiles.transit(), true);
        } else {
            fallbackPickup = true;
            approachPose = fallbackPickupPose;
            note = "no pollen seen: fallback pickup";
            followTo(fallbackPickupPose, PathProfiles.pickup(), true);
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
        intake.stop();
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
                    intake.stop();
                    return true;
                }
                if (readyToFire(pose) || volleyTimer.seconds() > AIM_TIMEOUT_S) {
                    intake.shoot();
                    volley = Volley.FEEDING;
                    volleyTimer.reset();
                }
                return false;

            case FEEDING:
                if (volleyTimer.seconds() >= SHOT_FEED_S) {
                    intake.stop();
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
        followTo(target, profile, false);
    }

    /**
     * Straight line from wherever the robot is to target, heading blending to target's. With
     * requireHeading, pathDone() also waits for the heading to settle (see
     * PATH_HEADING_TOLERANCE_DEG).
     */
    protected void followTo(Pose target, Modifier[] profile, boolean requireHeading) {
        launch(PathProfiles.straight(follower.pose(), target, profile), target, requireHeading);
    }

    private void launch(Path path, Pose target, boolean requireHeading) {
        follower.follow(path);
        pathTarget = target;
        pathNeedsHeading = requireHeading;
        pathEndedNs = 0;
    }

    /**
     * true once the follower has finished its path (Pedro then holds the end pose), and for
     * paths launched with requireHeading, once the heading has settled or the settle timeout
     * has run out.
     */
    protected boolean pathDone() {
        if (follower.following()) {
            pathEndedNs = 0;
            return false;
        }
        if (!pathNeedsHeading || pathTarget == null) {
            return true;
        }
        if (pathEndedNs == 0) {
            pathEndedNs = System.nanoTime();
        }
        double error = Math.abs(Math.toDegrees(Angle.normalizeSigned(follower.pose().heading() - pathTarget.heading())));
        return error <= PATH_HEADING_TOLERANCE_DEG
                || (System.nanoTime() - pathEndedNs) / 1e9 >= PATH_SETTLE_TIMEOUT_S;
    }

    protected void setState(State next) {
        state = next;
        entering = true;
        stateTimer.reset();
    }

    public State getState() {
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
