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
     * The shooter fires POLLEN only, so there is one table and nothing switches it at run time.
     * What that has to buy us is determinism: the table must be a pure function of the
     * Ballistics constants, so re-solving it gives the same numbers every time. If it ever
     * depended on mutable state that drifts, the shots would drift with it and the regression
     * printed on the Driver Station would stop describing what the robot is doing.
     */
    @Test
    public void theTableIsAPureFunctionOfTheBallisticsConstants() {
        double[] first = ShotTable.RPM.clone();
        double[] firstHood = ShotTable.HOOD.clone();
        double[] firstRegression = ShotTable.REGRESSION.clone();
        double firstError = ShotTable.regressionErrorFraction;

        ShotTable.regenerate();
        ShotTable.regenerate();

        for (int i = 0; i < first.length; i++) {
            assertEquals("row " + ShotTable.DISTANCE_IN[i] + " in moved on a re-solve",
                    first[i], ShotTable.RPM[i], 1e-9);
            assertEquals("hood at " + ShotTable.DISTANCE_IN[i] + " in moved on a re-solve",
                    firstHood[i], ShotTable.HOOD[i], 1e-9);
        }
        for (int k = 0; k < firstRegression.length; k++) {
            assertEquals("regression coefficient " + k + " moved on a re-solve",
                    firstRegression[k], ShotTable.REGRESSION[k], 1e-9);
        }
        assertEquals("fit error moved on a re-solve", firstError, ShotTable.regressionErrorFraction, 1e-12);
        assertEquals("the ball is POLLEN", Field.POLLEN_DIAMETER_IN, Ballistics.BALL_DIAMETER_IN, 1e-9);
        assertEquals("the ball is POLLEN", Field.POLLEN_MASS_KG, Ballistics.BALL_MASS_KG, 1e-12);
        System.out.printf("POLLEN-only table is stable across re-solves; 48 in -> %.0f rpm%n", ShotTable.rpm(48));
    }

}
