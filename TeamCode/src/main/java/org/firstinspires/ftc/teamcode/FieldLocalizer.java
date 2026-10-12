package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * One field pose for the drivetrain and turret. Pinpoint fuses its internal IMU with both
 * odometry pods; Pedro exposes field X/Y velocity in inches/second and CCW yaw in radians/second.
 * This wrapper rejects unhealthy samples and makes a pose reset also reset the motion state.
 */
public final class FieldLocalizer implements Localizer {
    private static final long MAX_SAMPLE_AGE_NS = 250_000_000L;

    private final Localizer source;
    private final Supplier<GoBildaPinpointDriver.DeviceStatus> deviceStatus;
    private final LongSupplier nanoTime;
    private MotionState motion = MotionState.zero();
    private Pose pendingPose;
    private boolean ready;
    private long lastGoodSampleNs;
    private String status = "Waiting for Pinpoint";

    public FieldLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());
        source = new PinpointLocalizer(hardwareMap, config);
        deviceStatus = pinpoint::getDeviceStatus;
        nanoTime = System::nanoTime;
        status = deviceStatus.get().toString();
    }

    /** Package-private dependency seam for deterministic sensor/reset tests. */
    FieldLocalizer(Localizer source, Supplier<GoBildaPinpointDriver.DeviceStatus> deviceStatus,
                   LongSupplier nanoTime) {
        this.source = source;
        this.deviceStatus = deviceStatus;
        this.nanoTime = nanoTime;
    }

    /**
     * Re-anchor X, Y and IMU heading to a known field pose. This does not zero the turret encoder
     * or recalibrate the IMU. Calibration at INIT must finish before sending a pending anchor.
     * A fresh sensor sample is required before automatic aiming is ready again.
     */
    @Override
    public void setPose(Pose pose) {
        if (!isFinite(pose)) {
            throw new IllegalArgumentException("Field pose must contain finite X, Y and heading");
        }
        pendingPose = pose;
        motion = MotionState.ofVelocity(pose, Velocity.zero());
        ready = false;
        lastGoodSampleNs = 0;
        status = "Pose reset: waiting for a fresh sample";
        if (deviceStatus.get() == GoBildaPinpointDriver.DeviceStatus.READY) {
            applyPendingPose();
        }
    }

    @Override
    public void update() {
        try {
            source.update();
            GoBildaPinpointDriver.DeviceStatus currentStatus = deviceStatus.get();
            if (currentStatus != GoBildaPinpointDriver.DeviceStatus.READY) {
                reject(currentStatus.toString());
                return;
            }
            if (pendingPose != null) {
                applyPendingPose();
                // This read predates the position write; never publish its old velocity/heading.
                return;
            }
            MotionState sample = source.state();
            if (sample == null || !isFinite(sample.pose()) || !isFinite(sample.velocity())) {
                reject("Invalid Pinpoint pose/velocity");
                return;
            }
            motion = sample;
            lastGoodSampleNs = nanoTime.getAsLong();
            ready = true;
            status = "READY";
        } catch (RuntimeException failure) {
            reject("Pinpoint read/write error: " + failure.getClass().getSimpleName());
        }
    }

    private void applyPendingPose() {
        try {
            source.setPose(pendingPose);
            pendingPose = null;
            ready = false;
            status = "Pose reset: waiting for a fresh sample";
        } catch (RuntimeException failure) {
            // Retain the requested anchor so a recovered device receives it before aiming resumes.
            reject("Pinpoint pose write error: " + failure.getClass().getSimpleName());
        }
    }

    private void reject(String reason) {
        motion = MotionState.ofVelocity(motion.pose(), Velocity.zero());
        ready = false;
        status = reason;
    }

    @Override
    public MotionState state() {
        return motion;
    }

    /** Recalibrate only while stationary. Driver position resets should use setPose instead. */
    @Override
    public void reset() {
        pendingPose = motion.pose();
        reject("Calibrating Pinpoint: keep robot stationary");
        source.reset();
    }

    public boolean isReady() {
        return ready && nanoTime.getAsLong() - lastGoodSampleNs <= MAX_SAMPLE_AGE_NS;
    }

    public String status() {
        return ready && !isReady() ? "Stale Pinpoint sample" : status;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Localization", status());
        Velocity velocity = motion.velocity();
        telemetry.addData("Field velocity", "x %.1f  y %.1f in/s  turn %.1f deg/s",
                velocity.vx, velocity.vy, Math.toDegrees(velocity.omega));
    }

    public static boolean isFinite(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static boolean isFinite(Velocity velocity) {
        return velocity != null && Double.isFinite(velocity.vx) && Double.isFinite(velocity.vy)
                && Double.isFinite(velocity.omega);
    }
}
