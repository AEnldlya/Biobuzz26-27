package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Ballistics;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.ShotTable;
import org.junit.Test;

/**
 * The shooting chain on its own: the physics-generated shot table and its RPM regression,
 * checked by flying POLLEN through the same physics into the tilted 20 x 14 in CELL mouth
 * from a sweep of distances, and by checking the flywheel model (4 mm compression) is
 * self-consistent.
 */
public class ShootingPhysicsTest {
    @Test
    public void regressionFitsThePhysicsTable() {
        System.out.println(ShotTable.describe());
        assertEquals("every distance in the table should be solvable at its hood angle", 0, ShotTable.unsolvedRows);
        assertTrue("regression should fit the physics table within 2 %, got " + ShotTable.regressionErrorFraction * 100 + " %",
                ShotTable.regressionErrorFraction < 0.02);
        for (int i = 1; i < ShotTable.DISTANCE_IN.length; i++) {
            assertTrue("rpm should rise with distance", ShotTable.RPM[i] > ShotTable.RPM[i - 1]);
        }
        double transfer = Ballistics.transferRatio();
        assertTrue("transfer ratio should be below the no-slip ideal", transfer < Ballistics.IDEAL_TRANSFER);
        assertTrue("4 mm of compression should give most of the grip", transfer > 0.6 * Ballistics.IDEAL_TRANSFER);
        assertEquals(Ballistics.exitSpeedInS(3000), Ballistics.exitSpeedInS(Ballistics.rpmForExitSpeed(Ballistics.exitSpeedInS(3000))), 1e-6);
        assertEquals("POLLEN by default", Field.POLLEN_DIAMETER_IN, Ballistics.BALL_DIAMETER_IN, 1e-9);
    }

    /** robot at distance d from the mouth, along the line from the mouth toward the alliance corner, facing it */
    private static void placeAt(SimWorld world, Field.CellMouth mouth, double d) {
        double ux = 0 - mouth.cx, uy = 0 - mouth.cy;
        double len = Math.hypot(ux, uy);
        ux /= len;
        uy /= len;
        double x = mouth.cx + ux * d, y = mouth.cy + uy * d;
        world.robot.setPose(new Pose(x, y, Math.atan2(mouth.cy - y, mouth.cx - x)));
    }

    @Test
    public void tableShotsDropIntoTheCellFromEveryDistance() {
        Field.CellMouth mouth = Field.cellMouth(Alliance.RED, Field.startingUpCell(Alliance.RED));
        double worst = 0;
        int missed = 0;
        for (double d = 18; d <= 78; d += 6) {
            // a fresh HIVE per shot: 3 POLLEN on top of the 3 staged NECTAR tip it, and a
            // tipping HIVE voids further shots (tested separately in AutoSimTest)
            SimWorld world = new SimWorld(Alliance.RED, 0.0);
            placeAt(world, mouth, d);
            world.hood.setPosition(ShotTable.hood(d));
            SimWorld.Shot shot = world.testShot(ShotTable.rpm(d));
            System.out.printf("%4.0f in: %5.0f rpm (%.0f in/s at %.1f deg) -> %s, %.1f in from the mouth centre, crossed at z %.1f%n",
                    d, shot.rpm, shot.exitSpeedInS, shot.exitAngleDeg, shot.scored ? "IN" : "MISS", shot.missIn, shot.landZ);
            worst = Math.max(worst, shot.missIn);
            if (!shot.scored) {
                missed++;
            }
        }
        assertEquals("every table shot should drop in", 0, missed);
        assertTrue("regression shots should be well inside the mouth, worst " + worst + " in", worst < 2.0);
    }

    @Test
    public void wrongSpeedMisses() {
        Field.CellMouth mouth = Field.cellMouth(Alliance.RED, Field.startingUpCell(Alliance.RED));
        double d = 48;
        SimWorld slowWorld = new SimWorld(Alliance.RED, 0.0);
        placeAt(slowWorld, mouth, d);
        slowWorld.hood.setPosition(ShotTable.hood(d));
        SimWorld.Shot slow = slowWorld.testShot(ShotTable.rpm(d) * 0.8);
        SimWorld fastWorld = new SimWorld(Alliance.RED, 0.0);
        placeAt(fastWorld, mouth, d);
        fastWorld.hood.setPosition(ShotTable.hood(d));
        SimWorld.Shot fast = fastWorld.testShot(ShotTable.rpm(d) * 1.25);
        System.out.printf("48 in: 80%% rpm miss %.1f in, 125%% rpm miss %.1f in%n", slow.missIn, fast.missIn);
        assertTrue("20 % slow should fall short", !slow.scored);
        assertTrue("25 % fast should overshoot", !fast.scored);
    }

    /**
     * POLLEN (2.8 in, 24.9 g) and NECTAR (3.62 in, 41.3 g) turn out to have almost the same
     * drag area per unit mass (0.1596 vs 0.1608 m^2/kg), so one RPM table serves both: no
     * re-tuning when the shooter is fed NECTAR. This pins that down so a future change to the
     * published masses shows up here.
     */
    @Test
    public void nectarFliesLikePollen() {
        double pollenRpm = ShotTable.rpm(48);
        double pollenRatio = dragAreaPerMass(Field.POLLEN_DIAMETER_IN, Field.POLLEN_MASS_KG);
        double nectarRatio = dragAreaPerMass(Field.NECTAR_DIAMETER_IN, Field.NECTAR_MASS_KG);
        Ballistics.setNectar(true);
        double nectarRpm;
        try {
            nectarRpm = ShotTable.rpm(48);
        } finally {
            Ballistics.setNectar(false);
        }
        System.out.printf("48 in: POLLEN %.0f rpm, NECTAR %.0f rpm (drag area/mass %.4f vs %.4f m^2/kg)%n",
                pollenRpm, nectarRpm, pollenRatio, nectarRatio);
        assertEquals("drag area per mass should be within 2 %", 1.0, nectarRatio / pollenRatio, 0.02);
        assertEquals("so the same RPM works for both, within 1 %", 1.0, nectarRpm / pollenRpm, 0.01);
        assertEquals("switching back restores the POLLEN table", pollenRpm, ShotTable.rpm(48), 1e-6);
        assertEquals("and the POLLEN ball", Field.POLLEN_DIAMETER_IN, Ballistics.BALL_DIAMETER_IN, 1e-9);
    }

    /**
     * Switching balls happens on a button in the middle of a match, so it must not re-solve the
     * table: every row bisects for an exit speed and every trial flies the ball in 2 ms steps,
     * which costs a fifth of a second on a Control Hub - one frozen loop with the turret and
     * flywheel not updating. Both tables are solved when ShotTable loads and the switch is a
     * copy. The bound is deliberately 1000x the real cost so it only fires if someone puts the
     * solve back on the button.
     */
    @Test
    public void switchingBallsMidMatchDoesNotResolveTheTable() {
        ShotTable.regenerate();                 // warm every class and JIT path first
        Ballistics.setNectar(false);
        double pollenRpm = ShotTable.rpm(48);

        try {
            long start = System.nanoTime();
            Ballistics.setNectar(true);
            double toNectarMs = (System.nanoTime() - start) / 1e6;
            double nectarRpm = ShotTable.rpm(48);

            start = System.nanoTime();
            Ballistics.setNectar(false);
            double backMs = (System.nanoTime() - start) / 1e6;

            System.out.printf("ball switch: %.3f ms out, %.3f ms back%n", toNectarMs, backMs);
            assertTrue("switching to NECTAR re-solved the table (" + toNectarMs + " ms)", toNectarMs < 2.0);
            assertTrue("switching back re-solved the table (" + backMs + " ms)", backMs < 2.0);
            assertTrue("the switch must still leave a usable NECTAR table, got " + nectarRpm,
                    nectarRpm > 1000 && nectarRpm < 6000);
            assertEquals("and switching back must restore POLLEN exactly", pollenRpm, ShotTable.rpm(48), 1e-9);
        } finally {
            Ballistics.setNectar(false);
        }
    }

    /** The cached table has to be what a full solve for that ball would have produced. */
    @Test
    public void theCachedTableMatchesAFullSolve() {
        try {
            Ballistics.setNectar(true);
            double[] cached = new double[ShotTable.DISTANCE_IN.length];
            System.arraycopy(ShotTable.RPM, 0, cached, 0, cached.length);
            double cachedError = ShotTable.regressionErrorFraction;
            int cachedUnsolved = ShotTable.unsolvedRows;

            ShotTable.regenerate();             // re-solve with NECTAR selected
            for (int i = 0; i < cached.length; i++) {
                assertEquals("row " + ShotTable.DISTANCE_IN[i] + " in differs from a fresh solve",
                        cached[i], ShotTable.RPM[i], 1e-9);
            }
            assertEquals("fit error differs from a fresh solve", cachedError, ShotTable.regressionErrorFraction, 1e-12);
            assertEquals("unsolved rows differ from a fresh solve", cachedUnsolved, ShotTable.unsolvedRows);
        } finally {
            Ballistics.setNectar(false);
        }
    }

    private static double dragAreaPerMass(double diameterIn, double massKg) {
        double rM = diameterIn * 0.0254 / 2.0;
        return Math.PI * rM * rM / massKg;
    }
}
