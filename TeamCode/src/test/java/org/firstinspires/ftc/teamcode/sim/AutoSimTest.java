package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.BlueAuto;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.RedAuto;
import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.junit.Test;

import java.io.File;

/**
 * Runs the real RED AUTO / BLUE AUTO OpModes against the simulated robot for a full 30 s
 * (real time, the code uses wall-clock timers) and checks the whole chain: Pedro drives the
 * placeholder paths, the turret stays on the CELL, the flywheel is at speed when the blocker
 * opens, the preload scores, the Limelight finds the biggest purple pile, the intake picks it
 * up, the robot comes back, shoots again, and parks.
 *
 * The replay is written to TeamCode/build/sim/auto_red.html (open it in a browser).
 */
public class AutoSimTest {
    private static void placePollen(SimWorld world, Alliance alliance) {
        // written for red and mirrored: a pile of four purple ahead of the scan pose, a lone
        // purple off to the side, and a green pile that must be ignored
        double[][] purple = {{20, 68}, {24, 70}, {28, 67}, {23, 73}, {8, 62}};
        double[][] green = {{34, 62}, {37, 64}, {36, 60}};
        for (double[] p : purple) {
            Pose m = alliance.fromRed(new Pose(p[0], p[1]));
            world.addPollen(Pollen.PURPLE, m.x(), m.y());
        }
        for (double[] p : green) {
            Pose m = alliance.fromRed(new Pose(p[0], p[1]));
            world.addPollen(Pollen.GREEN, m.x(), m.y());
        }
    }

    private static SimWorld run(AutoBase auto, Alliance alliance, String reportName) throws Exception {
        SimWorld world = new SimWorld(alliance, 0.0);
        placePollen(world, alliance);
        world.ballsInRobot = AutoBase.PRELOAD_BALLS;

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
            world.setLabel(state.name());
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
        System.out.printf("shots %d scored %d collected %d final pose %s%n", world.shotsFired(), world.ballsScored,
                world.collectedCount(), world.robot.truePose());
        for (SimWorld.Shot s : world.shots) {
            System.out.printf("  shot %.2f s rpm %.0f turret %.1f aim %.1f miss %.1f in %s%n", s.timeS, s.rpm,
                    s.turretDeg, s.trueAimDeg, s.missIn, s.scored ? "SCORED" : (s.dribbled ? "DRIBBLED" : "miss"));
        }
        return world;
    }

    private static void check(SimWorld world, AutoBase auto, Alliance alliance) {
        assertEquals("auto should finish parked", AutoBase.State.DONE, auto.getState());
        assertTrue("preload volley not fired: " + world.shotsFired(), world.shotsFired() >= AutoBase.PRELOAD_BALLS);
        for (SimWorld.Shot s : world.shots) {
            double err = Math.abs(s.turretDeg - s.trueAimDeg);
            assertTrue("turret off target at a shot: " + err + " deg", err <= 3.0);
        }
        assertTrue("preload should score: " + world.ballsScored, world.ballsScored >= 2);
        assertTrue("should collect pollen from the purple pile: " + world.collectedCount(), world.collectedCount() >= 2);
        for (SimWorld.PollenPiece p : world.pollen) {
            assertTrue("collected the wrong colour", !p.collected || p.color == Pollen.PURPLE);
        }
        assertTrue("should shoot a second volley: " + world.shotsFired(), world.shotsFired() >= AutoBase.PRELOAD_BALLS + 1);
        Pose park = alliance.fromRed(Field.RED_PARK);
        Pose end = world.robot.truePose();
        assertTrue("should end near the park pose: " + end, Math.hypot(end.x() - park.x(), end.y() - park.y()) <= 6.0);
    }

    @Test
    public void redAutoScoresCollectsAndParks() throws Exception {
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

    /** keeps OpMode's protected members reachable for the harness */
    static OpMode asOpMode(AutoBase auto) {
        return auto;
    }
}
