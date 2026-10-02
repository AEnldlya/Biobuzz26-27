package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

/** PollenCamera backed by a Limelight 3A running colour pipelines. */
public class LimelightPollenCamera implements PollenCamera {
    private final Limelight3A limelight;

    public LimelightPollenCamera(HardwareMap hardwareMap, String name) {
        Limelight3A found;
        try {
            found = hardwareMap.get(Limelight3A.class, name);
        } catch (RuntimeException e) {
            found = null;
        }
        limelight = found;
    }

    @Override
    public boolean isConnected() {
        return limelight != null;
    }

    @Override
    public void start(int pipelineIndex, int pollRateHz) {
        if (limelight == null) {
            return;
        }
        limelight.setPollRateHz(pollRateHz);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    @Override
    public void switchPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    @Override
    public Frame latest() {
        if (limelight == null) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        List<Blob> blobs = new ArrayList<>();
        List<LLResultTypes.ColorResult> colors = result.getColorResults();
        if (colors != null) {
            for (LLResultTypes.ColorResult c : colors) {
                blobs.add(new Blob(c.getTargetXDegrees(), c.getTargetYDegrees(), c.getTargetArea()));
            }
        }
        return new Frame(result.getPipelineIndex(), result.getTimestamp(), result.getStaleness(),
                result.getCaptureLatency() + result.getTargetingLatency(), blobs);
    }
}
