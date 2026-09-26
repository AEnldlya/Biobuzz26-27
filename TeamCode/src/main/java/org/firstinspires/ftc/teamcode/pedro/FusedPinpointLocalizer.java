package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Custom Pedro localizer: goBILDA Pinpoint (two dead wheels + its own IMU) with the Control Hub
 * IMU as a heading backstop, plus a short pose history for camera latency compensation.
 *
 * Normal operation is exactly Pedro's PinpointLocalizer; this class only adds:
 * <ul>
 *   <li>Pinpoint health monitoring. The Pinpoint's device status comes back in the same bulk
 *       read Pedro already does, so this costs nothing. While the Pinpoint is READY the offset
 *       between its heading and the hub IMU yaw is tracked (the two agree up to a constant
 *       once setPose() has been called).</li>
 *   <li>Hub IMU fallback. If the Pinpoint faults (pod unplugged, IMU runaway, bad I2C read)
 *       the heading keeps coming from the hub IMU + that offset, so the turret keeps tracking
 *       the goal, and the position freezes at the last good value instead of going to garbage.
 *       Velocity is reported as zero in fallback.</li>
 *   <li>poseAt(nanoTime): the robot pose at an earlier instant, used to place Limelight
 *       detections on the field using the pose from when the frame was captured, not the pose
 *       now.</li>
 * </ul>
 *
 * The hub IMU is only read every IMU_POLL_MS while the Pinpoint is healthy (an IMU read is a
 * separate I2C transaction, not part of the bulk read), and every loop in fallback.
 */
public class FusedPinpointLocalizer implements Localizer, PoseHistory {
    /** Control Hub IMU name in the robot configuration; leave the hub's built-in "imu". */
    public static String IMU_NAME = "imu";
    /** How the Control Hub is mounted; only matters for the fallback heading. */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static boolean USE_HUB_IMU_FALLBACK = true;
    public static long IMU_POLL_MS = 100;
    /** weight of each new heading-offset sample while healthy (slow: the offset is constant) */
    public static double OFFSET_FILTER = 0.05;
    /** how long a fault must persist before switching to the hub IMU, filters one bad read */
    public static long FAULT_DEBOUNCE_MS = 60;
    /** pose history length in ms; Limelight results are at most a few hundred ms old */
    public static long HISTORY_MS = 1500;

    private static final class TimedPose {
        final long ns;
        final Pose pose;

        TimedPose(long ns, Pose pose) {
            this.ns = ns;
            this.pose = pose;
        }
    }

    private final PinpointLocalizer pinpoint;
    private final GoBildaPinpointDriver driver;
    private final IMU imu;
    private final ArrayDeque<TimedPose> history = new ArrayDeque<>();

    private MotionState state = MotionState.zero();
    private Pose lastGoodPose = Pose.zero();
    private GoBildaPinpointDriver.DeviceStatus status = GoBildaPinpointDriver.DeviceStatus.NOT_READY;
    private double imuYawRad = 0.0;
    private double imuOffsetRad = 0.0;
    private boolean offsetValid = false;
    private boolean fallback = false;
    private long lastImuReadMs = 0;
    private long faultSinceMs = 0;
    private Pose pendingPose = null;

    public FusedPinpointLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        pinpoint = new PinpointLocalizer(hardwareMap, config);
        // same device instance Pedro's localizer talks to: status is cached by its update()
        driver = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());

        IMU found = null;
        if (USE_HUB_IMU_FALLBACK) {
            try {
                found = hardwareMap.get(IMU.class, IMU_NAME);
                found.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_FACING, USB_FACING)));
            } catch (RuntimeException e) {
                // not in the configuration: run on the Pinpoint alone
                found = null;
            }
        }
        imu = found;
        state = pinpoint.state();
        lastGoodPose = state.pose();
    }

    @Override
    public void setPose(Pose pose) {
        pinpoint.setPose(pose);
        state = MotionState.ofVelocity(pose, Velocity.zero());
        lastGoodPose = pose;
        history.clear();
        fallback = false;
        faultSinceMs = 0;
        // A position write to a Pinpoint that is still calibrating its IMU can be dropped, and
        // the robot would then start the match believing it is at the field origin: the turret
        // aims at nothing and the first path drives off the tiles. Pedro only recalibrates on
        // reset(), which no match OpMode calls, so the device is normally READY here - but the
        // cost of being wrong is the whole match, so remember the pose and put it back on the
        // first healthy update if it was not.
        pendingPose = status == GoBildaPinpointDriver.DeviceStatus.READY ? null : pose;
        if (imu != null) {
            readImu(System.currentTimeMillis());
            imuOffsetRad = Angle.normalizeSigned(pose.heading() - imuYawRad);
            offsetValid = true;
        }
    }

    @Override
    public void update() {
        pinpoint.update();
        status = driver.getDeviceStatus();
        long nowMs = System.currentTimeMillis();
        boolean healthy = status == GoBildaPinpointDriver.DeviceStatus.READY;

        if (healthy) {
            faultSinceMs = 0;
            fallback = false;
            if (pendingPose != null) {
                Pose retry = pendingPose;
                pendingPose = null;
                setPose(retry);
            }
            state = pinpoint.state();
            lastGoodPose = state.pose();
            if (imu != null && nowMs - lastImuReadMs >= IMU_POLL_MS) {
                readImu(nowMs);
                double sample = Angle.normalizeSigned(state.pose().heading() - imuYawRad);
                if (offsetValid) {
                    imuOffsetRad = Angle.normalize(imuOffsetRad
                            + OFFSET_FILTER * Angle.normalizeSigned(sample - imuOffsetRad));
                } else {
                    imuOffsetRad = sample;
                    offsetValid = true;
                }
            }
        } else {
            if (faultSinceMs == 0) {
                faultSinceMs = nowMs;
            }
            boolean canFallBack = imu != null && offsetValid
                    && status != GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                    && nowMs - faultSinceMs >= FAULT_DEBOUNCE_MS;
            if (canFallBack) {
                fallback = true;
                readImu(nowMs);
                double heading = Angle.normalize(imuYawRad + imuOffsetRad);
                state = MotionState.ofVelocity(lastGoodPose.withHeading(heading), Velocity.zero());
            } else {
                // brief glitch or still calibrating: keep the last good pose, no velocity
                state = MotionState.ofVelocity(lastGoodPose, Velocity.zero());
            }
        }

        long nowNs = System.nanoTime();
        history.addLast(new TimedPose(nowNs, state.pose()));
        long oldest = nowNs - HISTORY_MS * 1_000_000L;
        while (history.size() > 2 && history.peekFirst().ns < oldest) {
            history.pollFirst();
        }
    }

    private void readImu(long nowMs) {
        imuYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastImuReadMs = nowMs;
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void reset() {
        pinpoint.reset();
        state = pinpoint.state();
        lastGoodPose = state.pose();
        offsetValid = false;
        fallback = false;
        history.clear();
    }

    /**
     * The pose the robot had at System.nanoTime() == nanoTime, linearly interpolated from the
     * history. Times before the history return its oldest pose; times after it return the
     * current pose.
     */
    @Override
    public Pose poseAt(long nanoTime) {
        if (history.isEmpty()) {
            return state.pose();
        }
        TimedPose before = null;
        for (TimedPose sample : history) {
            if (sample.ns >= nanoTime) {
                if (before == null) {
                    return sample.pose;
                }
                double span = sample.ns - before.ns;
                double t = span <= 0 ? 1.0 : (nanoTime - before.ns) / span;
                return Pose.interpolate(before.pose, sample.pose, Math.max(0.0, Math.min(1.0, t)));
            }
            before = sample;
        }
        return state.pose();
    }

    public GoBildaPinpointDriver.DeviceStatus status() {
        return status;
    }

    /** true while the heading is coming from the hub IMU because the Pinpoint faulted */
    public boolean usingImuFallback() {
        return fallback;
    }

    public boolean hasHubImu() {
        return imu != null;
    }

    public double imuOffsetDeg() {
        return Math.toDegrees(imuOffsetRad);
    }

    /** true while a start pose is still waiting for the Pinpoint to come READY */
    public boolean poseWritePending() {
        return pendingPose != null;
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new LinkedHashMap<>();
        map.put("pinpoint", status);
        map.put("imuFallback", fallback);
        map.put("hubImu", imu != null);
        map.put("imuOffsetDeg", Math.toDegrees(imuOffsetRad));
        map.put("poseWritePending", pendingPose != null);
        return map;
    }
}
