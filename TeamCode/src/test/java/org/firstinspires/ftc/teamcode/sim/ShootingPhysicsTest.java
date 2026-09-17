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
 * checked by flying balls through the same physics into the tilted CELL opening from a sweep
 * of distances, and by checking the flywheel model (4 mm compression) is self-consistent.
 */
public class ShootingPhysicsTest {
    @Test
    public void regressionFitsThePhysicsTable() {
        System.out.println(ShotTable.describe());
        assertEquals("every distance in the table should be solvable at its hood angle", 0, ShotTable.unsolvedRows);
        assertTrue("regression should fit the physics table within 2 %, got " + ShotTable.regressionErrorFraction * 100 + " %",
                ShotTable.regressionErrorFraction < 0.02);
        // monotone: further needs faster
        for (int i = 1; i < ShotTable.DISTANCE_IN.length; i++) {
            assertTrue("rpm should rise with distance", ShotTable.RPM[i] > ShotTable.RPM[i - 1]);
        }
        // sanity on the flywheel model at 4 mm compression
        double transfer = Ballistics.transferRatio();
        assertTrue("transfer ratio should be below the no-slip ideal", transfer < Ballistics.IDEAL_TRANSFER);
        assertTrue("4 mm of compression should give most of the grip", transfer > 0.6 * Ballistics.IDEAL_TRANSFER);
        assertEquals(Ballistics.exitSpeedInS(3000), Ballistics.exitSpeedInS(Ballistics.rpmForExitSpeed(Ballistics.exitSpeedInS(3000))), 1e-6);
    }

    @Test
    public void tableShotsDropIntoTheCellFromEveryDistance() {
        SimWorld world = new SimWorld(Alliance.RED, 0.0);
        Pose cell = Field.cellAimPoint(Alliance.RED, Field.startingUpCell(Alliance.RED));
        double worst = 0;
        int missed = 0;
        for (double d = 24; d <= 132; d += 12) {
            // straight south of the CELL (opening side), facing it, turret straight ahead
            world.robot.setPose(new Pose(cell.x(), cell.y() - d, Math.toRadians(90)));
            world.hood.setPosition(ShotTable.hood(d));
            SimWorld.Shot shot = world.testShot(ShotTable.rpm(d));
            System.out.printf("%4.0f in: %5.0f rpm (%.0f in/s at %.1f deg) -> %s, %.1f in from the opening centre, crossed at z %.1f%n",
                    d, shot.rpm, shot.exitSpeedInS, shot.exitAngleDeg, shot.scored ? "IN" : "MISS", shot.missIn, shot.landZ);
            worst = Math.max(worst, shot.missIn);
            if (!shot.scored) {
                missed++;
            }
        }
        assertEquals("every table shot should drop in", 0, missed);
        assertTrue("regression shots should be well inside the opening, worst " + worst + " in",
                worst < Field.CELL_OPENING_RADIUS_IN * 0.6);
    }

    @Test
    public void wrongSpeedMisses() {
        SimWorld world = new SimWorld(Alliance.RED, 0.0);
        Pose cell = Field.cellAimPoint(Alliance.RED, Field.startingUpCell(Alliance.RED));
        double d = 72;
        world.robot.setPose(new Pose(cell.x(), cell.y() - d, Math.toRadians(90)));
        world.hood.setPosition(ShotTable.hood(d));
        SimWorld.Shot slow = world.testShot(ShotTable.rpm(d) * 0.8);
        SimWorld.Shot fast = world.testShot(ShotTable.rpm(d) * 1.25);
        System.out.printf("72 in: 80%% rpm miss %.1f in, 125%% rpm miss %.1f in%n", slow.missIn, fast.missIn);
        assertTrue("20 % slow should fall short", !slow.scored);
        assertTrue("25 % fast should overshoot", !fast.scored);
    }
}
