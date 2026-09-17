package org.firstinspires.ftc.teamcode.vision;

/**
 * The pollen (game piece) colours the Limelight can pick out, one colour pipeline each.
 *
 * Build each pipeline in the Limelight web UI (Pipeline Type: Color, tune the HSV thresholds
 * on a real piece under field lighting, set a minimum area, and raise "max targets" so every
 * blob comes back, not only the biggest). The code only cares about the pipeline index; the
 * names are the slots, rename them to the real BIOBUZZ pollen colours.
 */
public enum Pollen {
    PURPLE(1),
    GREEN(2),
    YELLOW(3);

    public final int pipeline;

    Pollen(int pipeline) {
        this.pipeline = pipeline;
    }
}
