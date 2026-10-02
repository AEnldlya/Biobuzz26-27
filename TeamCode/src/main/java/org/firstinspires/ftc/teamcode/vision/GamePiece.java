package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Field;

/**
 * BIOBUZZ scoring elements the Limelight can pick out, one colour pipeline each.
 *
 * POLLEN are yellow 2.8 in balls that either alliance may use; NECTAR are red or blue 3.62 in
 * balls that only their alliance may control (G408). Build one Color pipeline per entry in the
 * Limelight web UI (tune the HSV thresholds on a real piece under field lighting, set a
 * minimum area, raise "max targets" so every blob comes back) and put its index here.
 */
public enum GamePiece {
    POLLEN(1, Field.POLLEN_DIAMETER_IN, Field.POLLEN_MASS_KG),
    RED_NECTAR(2, Field.NECTAR_DIAMETER_IN, Field.NECTAR_MASS_KG),
    BLUE_NECTAR(3, Field.NECTAR_DIAMETER_IN, Field.NECTAR_MASS_KG);

    public final int pipeline;
    public final double diameterIn;
    public final double massKg;

    GamePiece(int pipeline, double diameterIn, double massKg) {
        this.pipeline = pipeline;
        this.diameterIn = diameterIn;
        this.massKg = massKg;
    }

    public double radiusIn() {
        return diameterIn / 2.0;
    }

    public boolean isPollen() {
        return this == POLLEN;
    }

    public boolean isNectar() {
        return this != POLLEN;
    }

    /** how many POLLEN this piece weighs, for HIVE tipping */
    public double pollenEquivalents() {
        return massKg / Field.POLLEN_MASS_KG;
    }

    public static GamePiece nectarOf(Alliance alliance) {
        return alliance == Alliance.RED ? RED_NECTAR : BLUE_NECTAR;
    }

    /** may this alliance's robot CONTROL the piece (G408: never the opponent's NECTAR) */
    public boolean controllableBy(Alliance alliance) {
        return this == POLLEN || this == nectarOf(alliance);
    }
}
