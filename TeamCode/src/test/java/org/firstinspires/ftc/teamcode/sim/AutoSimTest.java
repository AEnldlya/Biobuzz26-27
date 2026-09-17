package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.BlueAuto;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.RedAuto;
import org.firstinspires.ftc.teamcode.vision.GamePiece;
import org.junit.Test;

import java.io.File;

/**
 * Runs the real RED AUTO / BLUE AUTO OpModes on the BIOBUZZ field for a full 30 s (real time,
 * the code uses wall-clock timers) and scores it like the rules do: the robot starts on its
 * wall with 4 POLLEN, the up CELL holds 3 NECTAR, 3 POLLEN tip the HIVE (20), the GARDEN has
 * 4 POLLEN in the corner, LEAVE is 3, PARK in the LOADING ZONE is 5, and elements left in the
 * up CELL are 2 each.
 *
 * The replay is written to TeamCode/build/sim/auto_red.html (open it in a browser).
 */
public class AutoSimTest {
    private static SimWorld run(AutoBase auto, Alliance alliance, String reportName) throws Exception {
        SimWorld world = new SimWorld(alliance, 0.0).stagePerRules();
        // an opponent NECTAR lying near our GARDEN must be ignored (G408)
        Pose decoy = alliance.fromRed(new Pose(16, 6));
        world.addPiece(GamePiece.nectarOf(alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED), decoy.x(), decoy.y());
        world.ballsInRobot = Field.PRELOAD_POLLEN;

        auto.hardwareMap = world.hardwareMap;
        auto.telemetry = world.telemetry;
        auto.gamepad1 = new Gamepad();
        auto.gamepad2 = new Gamepad();

        auto.init();
        for (int i = 0; i < 20; i++) {
            auto.init_loop();
            Thread.sleep(10);
        }
        auto.start();
        long startNs = System.nanoTime();
        AutoBase.State lastState = null;
        while (System.nanoTime() - startNs < 31_000_000_000L) {
            auto.loop();
            AutoBase.State state = auto.getState();
            world.setLabel(state.name() + (world.tipping ? "  (HIVE tipping)" : "") + "  tips " + world.tips);
            if (state != lastState) {
                System.out.printf("%6.2f s  %s  %s%n", (System.nanoTime() - startNs) / 1e9, state, world.telemetry.get("Auto"));
                lastState = state;
            }
            if (state == AutoBase.State.DONE) {
                break;
            }
            Thread.sleep(6);
        }
        auto.stop();
        File html = world.report.write(reportName);
        System.out.println("replay: " + html.getAbsolutePath());
        System.out.printf("shots %d scored %d collected %d tips %d parked %s final pose %s%n", world.shotsFired(),
                world.ballsScored, world.collectedCount(), world.tips, world.parked(), world.robot.truePose());
        System.out.println("AUTO points: " + world.pointsBreakdown());
        for (SimWorld.Shot s : world.shots) {
            System.out.printf("  shot %.2f s at %s CELL, %.0f in, rpm %.0f turret %.1f aim %.1f miss %.1f in %s%n", s.timeS, s.cell,
                    s.distanceIn, s.rpm, s.turretDeg, s.trueAimDeg, s.missIn,
                    s.scored ? "SCORED" : (s.dribbled ? "DRIBBLED" : (s.duringTip ? "during tip" : "miss")));
        }
        return world;
    }

    private static void check(SimWorld world, AutoBase auto, Alliance alliance) {
        assertEquals("auto should finish parked", AutoBase.State.DONE, auto.getState());
        assertTrue("tip volley not fired: " + world.shotsFired(), world.shotsFired() >= AutoBase.TIP_VOLLEY_BALLS);
        for (int i = 0; i < AutoBase.TIP_VOLLEY_BALLS; i++) {
            assertTrue("preload shot " + i + " should score", world.shots.get(i).scored);
        }
        for (SimWorld.Shot s : world.shots) {
            double err = Math.abs(s.turretDeg - s.trueAimDeg);
            assertTrue("turret off target at a shot: " + err + " deg", err <= 3.0);
        }
        assertEquals("the HIVE should have tipped once", 1, world.tips);
        assertTrue("should collect POLLEN from the GARDEN: " + world.collectedCount(), world.collectedCount() >= 2);
        for (SimWorld.Piece p : world.pieces) {
            assertTrue("collected an opponent NECTAR", !p.collected || p.type.controllableBy(alliance));
        }
        boolean scoredAfterTip = false;
        for (SimWorld.Shot s : world.shots) {
            if (s.cell == Field.startingUpCell(alliance).flipped() && s.scored) {
                scoredAfterTip = true;
            }
        }
        assertTrue("should score into the new up CELL after the tip", scoredAfterTip);
        assertTrue("should end PARKED in the LOADING ZONE: " + world.robot.truePose(), world.parked());
        assertTrue("no ball should be wasted into a swinging HIVE: " + world.shotsFired() + " shots, "
                + world.ballsScored + " in", world.ballsScored >= 6);
        assertTrue("AUTO points too low: " + world.pointsBreakdown(), world.autoPoints() >= 3 + 20 + 5 + 2);
    }

    @Test
    public void redAutoTipsCollectsScoresAndParks() throws Exception {
        RedAuto auto = new RedAuto();
        SimWorld world = run(auto, Alliance.RED, "auto_red");
        check(world, auto, Alliance.RED);
    }

    @Test
    public void blueAutoMirrorsRed() throws Exception {
        BlueAuto auto = new BlueAuto();
        SimWorld world = run(auto, Alliance.BLUE, "auto_blue");
        check(world, auto, Alliance.BLUE);
    }
}
