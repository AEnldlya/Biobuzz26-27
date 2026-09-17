package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

/**
 * What PollenVision needs from a camera: colour blobs with angles and areas, per frame. The
 * real one is LimelightPollenCamera; the simulator supplies a virtual one.
 */
public interface PollenCamera {
    final class Blob {
        /** degrees right of the crosshair */
        public final double txDeg;
        /** degrees above the crosshair */
        public final double tyDeg;
        /** percent of the image */
        public final double area;

        public Blob(double txDeg, double tyDeg, double area) {
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
        }
    }

    final class Frame {
        public final int pipelineIndex;
        /** camera-side frame timestamp; only used to tell frames apart */
        public final double timestamp;
        /** ms since the frame's result arrived on the robot */
        public final long stalenessMs;
        /** camera capture + processing latency, ms */
        public final double latencyMs;
        public final List<Blob> blobs;

        public Frame(int pipelineIndex, double timestamp, long stalenessMs, double latencyMs, List<Blob> blobs) {
            this.pipelineIndex = pipelineIndex;
            this.timestamp = timestamp;
            this.stalenessMs = stalenessMs;
            this.latencyMs = latencyMs;
            this.blobs = blobs;
        }
    }

    boolean isConnected();

    void start(int pipelineIndex, int pollRateHz);

    void stop();

    void switchPipeline(int pipelineIndex);

    /** The newest frame, or null if none / not valid. */
    Frame latest();
}
