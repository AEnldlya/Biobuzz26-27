package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.firstinspires.ftc.teamcode.vision.PollenCamera;
import org.firstinspires.ftc.teamcode.vision.PollenVision;

import java.util.ArrayList;
import java.util.List;

/**
 * A virtual Limelight running colour pipelines. Every 1/fps seconds it looks at the world's
 * pollen from the TRUE robot pose of that instant, keeps the pieces of the active pipeline's
 * colour that fall inside the field of view, and turns each into a blob: tx / ty from the
 * camera mount geometry (the exact inverse of PollenVision.project) and an area that falls
 * off with the square of the range. The frame becomes visible latencyMs later, and a pipeline
 * switch takes switchMs, like the real camera.
 */
public class SimCamera implements PollenCamera {
    public double fps = 30.0;
    public double latencyMs = 35.0;
    public double switchMs = 250.0;
    public double halfFovXDeg = 27.0;
    public double halfFovYDeg = 20.5;
    public double areaAt12In = 3.0;
    public double noiseDeg = 0.15;

    private final SimWorld world;
    private boolean running = false;
    private int pipeline = 0;
    private int reportedPipeline = 0;
    private long switchDoneNs = 0;
    private long lastCaptureNs = 0;
    private int frameIndex = 0;
    private Frame pending = null;
    private long pendingReadyNs = 0;
    private Frame latest = null;
    private long latestArrivalNs = 0;
    private final java.util.Random random = new java.util.Random(11);

    public SimCamera(SimWorld world) {
        this.world = world;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public void start(int pipelineIndex, int pollRateHz) {
        switchPipeline(pipelineIndex);
        running = true;
    }

    @Override
    public void stop() {
        running = false;
    }

    @Override
    public void switchPipeline(int pipelineIndex) {
        if (pipelineIndex != pipeline) {
            pipeline = pipelineIndex;
            switchDoneNs = System.nanoTime() + (long) (switchMs * 1e6);
            latest = null;
            pending = null;
        }
    }

    /** Called by the world every step. */
    public void step() {
        if (!running) {
            return;
        }
        long now = System.nanoTime();
        if (now >= switchDoneNs) {
            reportedPipeline = pipeline;
        }
        if (pending != null && now >= pendingReadyNs) {
            latest = pending;
            latestArrivalNs = pendingReadyNs;
            pending = null;
        }
        if (now - lastCaptureNs >= 1e9 / fps && pending == null) {
            lastCaptureNs = now;
            pending = capture(now);
            pendingReadyNs = now + (long) (latencyMs * 1e6);
        }
    }

    private Frame capture(long now) {
        frameIndex++;
        Pose robot = world.robot.truePose();
        List<Blob> blobs = new ArrayList<>();
        if (reportedPipeline == pipeline) {
            for (SimWorld.PollenPiece piece : world.pollen) {
                if (piece.collected || piece.color.pipeline != reportedPipeline) {
                    continue;
                }
                Blob blob = see(robot, piece.x, piece.y);
                if (blob != null) {
                    blobs.add(blob);
                }
            }
        }
        return new Frame(reportedPipeline, frameIndex, 0, latencyMs, blobs);
    }

    /** The inverse of PollenVision.project: a field point to tx / ty / area, or null if out of view. */
    Blob see(Pose robot, double px, double py) {
        double dx = px - robot.x();
        double dy = py - robot.y();
        double cos = Math.cos(robot.heading());
        double sin = Math.sin(robot.heading());
        double forward = dx * cos + dy * sin - PollenVision.CAMERA_FORWARD_IN;
        double left = -dx * sin + dy * cos - PollenVision.CAMERA_LEFT_IN;
        double range = Math.hypot(forward, left);
        if (range < 3.0) {
            return null;
        }
        double bearing = Math.atan2(left, forward);
        double tx = PollenVision.CAMERA_YAW_DEG - Math.toDegrees(bearing);
        double drop = PollenVision.CAMERA_HEIGHT_IN - PollenVision.POLLEN_HEIGHT_IN;
        double below = Math.toDegrees(Math.atan2(drop, range));
        double ty = PollenVision.CAMERA_PITCH_DEG - below;
        if (Math.abs(tx) > halfFovXDeg || Math.abs(ty) > halfFovYDeg) {
            return null;
        }
        double area = areaAt12In * (12.0 / range) * (12.0 / range);
        return new Blob(tx + random.nextGaussian() * noiseDeg, ty + random.nextGaussian() * noiseDeg, area);
    }

    @Override
    public Frame latest() {
        if (latest == null) {
            return null;
        }
        long staleness = (System.nanoTime() - latestArrivalNs) / 1_000_000L;
        return new Frame(latest.pipelineIndex, latest.timestamp, staleness, latest.latencyMs, latest.blobs);
    }

    public int framesCaptured() {
        return frameIndex;
    }
}
