package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.Turret;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The real TeleOp OpMode on the simulated robot.
 *
 * AUTO is the same code every match; TELEOP is the code a driver is holding when the match is
 * on the line, and none of it - the stick mapping, the fire button, the hand-off from AUTO -
 * is exercised anywhere else. A flipped stick sign compiles, passes every other test, and
 * loses the match, so it is pinned down here.
 */
public class TeleOpSimTest {
    /** A started TeleOp OpMode with its two gamepads and the world it is driving. */
    private static final class Rig {
        final SimWorld world;
        final BiobuzzTeleOp opMode = new BiobuzzTeleOp();
        final Gamepad driver = new Gamepad();
        final Gamepad operator = new Gamepad();

        Rig(Alliance alliance, Pose startPose, Field.CellSide upCell, double turretDeg) throws Exception {
            // the ring really is at turretDeg: TELEOP inherits the hardware AUTO left behind
            world = new SimWorld(alliance, turretDeg).stagePerRules();
            world.upCell = upCell;
            // hand over from AUTO exactly as AutoBase leaves it
            RobotState.alliance = alliance;
            RobotState.pose = startPose;
            RobotState.turretAngleDeg = turretDeg;
            RobotState.upCell = upCell;

            opMode.hardwareMap = world.hardwareMap;
            opMode.telemetry = world.telemetry;
            opMode.gamepad1 = driver;
            opMode.gamepad2 = operator;
            opMode.init();
            for (int i = 0; i < 5; i++) {
                opMode.init_loop();
                Thread.sleep(6);
            }
            opMode.start();
        }

        /** Runs the OpMode for seconds of wall clock, calling watcher every loop. */
        void run(double seconds, Runnable watcher) throws Exception {
            long start = System.nanoTime();
            while ((System.nanoTime() - start) / 1e9 < seconds) {
                opMode.loop();
                world.setLabel(String.format("teleop  turret err %.2f", world.turretErrorDeg()));
                watcher.run();
                Thread.sleep(6);
            }
        }

        void run(double seconds) throws Exception {
            run(seconds, () -> { });
        }

        void sticksOff() {
            driver.left_stick_x = 0f;
            driver.left_stick_y = 0f;
            driver.right_stick_x = 0f;
        }
    }

    private static double p95(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted.get((int) Math.floor(0.95 * (sorted.size() - 1)));
    }

    /**
     * The stick mapping. Pedro's manual() takes (forward, strafe, turn) in the ROBOT frame with
     * +strafe to the left and +turn counter-clockwise, and the sticks report y down-positive, so
     * every one of the three is negated. Get one wrong and the robot drives sideways on command.
     */
    @Test
    public void sticksDriveTheRobotTheRightWay() throws Exception {
        Rig rig = new Rig(Alliance.RED, new Pose(72, 72, 0.0), Field.CellSide.AUDIENCE, 0.0);

        rig.driver.left_stick_y = -1.0f;         // push forward
        rig.run(0.8);
        rig.sticksOff();
        Pose forward = rig.world.robot.truePose();
        System.out.printf("teleop forward stick -> %s%n", forward);
        assertTrue("forward stick must drive toward the robot's front (+x at heading 0), got "
                + forward, forward.x() - 72 > 6.0);
        assertTrue("forward stick must not strafe: " + forward, Math.abs(forward.y() - 72) < 3.0);

        rig.run(0.5); // let it stop
        rig.world.robot.setPose(new Pose(72, 72, 0.0));
        rig.driver.left_stick_x = 1.0f;          // push right
        rig.run(0.8);
        rig.sticksOff();
        Pose right = rig.world.robot.truePose();
        System.out.printf("teleop right stick  -> %s%n", right);
        assertTrue("right on the left stick must strafe to the robot's right (-y at heading 0), got "
                + right, right.y() - 72 < -4.0);

        rig.run(0.5);
        rig.world.robot.setPose(new Pose(72, 72, 0.0));
        rig.driver.right_stick_x = 1.0f;         // turn right
        rig.run(0.6);
        rig.sticksOff();
        double heading = Math.toDegrees(Angle.normalizeSigned(rig.world.robot.truePose().heading()));
        System.out.printf("teleop turn stick   -> heading %.1f deg%n", heading);
        assertTrue("right on the right stick must turn the robot clockwise (heading down), got "
                + heading, heading < -20.0);

        rig.opMode.stop();
    }

    /** The driver spins the robot; the turret has to stay on the CELL through it. */
    @Test
    public void turretHoldsTheCellWhileTheDriverSpins() throws Exception {
        double lead = Turret.LEAD_GAIN;
        Turret.LEAD_GAIN = 0.0; // measure pure pointing, not the deliberate shoot-on-the-move lead
        try {
            Rig rig = new Rig(Alliance.RED, new Pose(40, 40, 0.0), Field.CellSide.AUDIENCE, 0.0);
            rig.run(1.5); // settle on target before moving

            List<Double> errors = new ArrayList<>();
            rig.driver.right_stick_x = 0.35f;
            rig.run(1.2, () -> errors.add(Math.abs(rig.world.turretErrorDeg())));
            rig.driver.right_stick_x = -0.35f;
            rig.driver.left_stick_y = -0.5f;
            rig.run(1.2, () -> errors.add(Math.abs(rig.world.turretErrorDeg())));
            rig.sticksOff();
            rig.opMode.stop();

            double worst = Collections.max(errors);
            System.out.printf("teleop turret while driving: p95 %.2f deg, max %.2f deg%n", p95(errors), worst);
            assertTrue("turret lost the CELL while the driver drove: p95 " + p95(errors), p95(errors) < 4.0);
            assertTrue("turret spike while the driver drove: " + worst, worst < 8.0);
        } finally {
            Turret.LEAD_GAIN = lead;
        }
    }

    /**
     * The intake/blocker mapping: the trigger stacks balls behind the blocker, the bumper opens
     * it and feeds, and letting go of the bumper closes it again rather than dribbling the rest.
     */
    @Test
    public void triggerIntakesAndBumperFires() throws Exception {
        Alliance alliance = Alliance.RED;
        Rig rig = new Rig(alliance, Field.scorePose(alliance, Field.startingUpCell(alliance)),
                Field.startingUpCell(alliance), -38.0);
        rig.world.ballsInRobot = 3;

        rig.driver.right_trigger = 1.0f;
        rig.run(0.4);
        System.out.printf("intake: motors %.2f / %.2f  blocker %.3f%n", rig.world.intakeMotor.getPower(),
                rig.world.transferMotor.getPower(), rig.world.blocker.getPosition());
        assertTrue("right trigger must run the intake motor", rig.world.intakeMotor.getPower() > 0.1);
        assertTrue("the transfer motor must run with the intake", rig.world.transferMotor.getPower() > 0.1);
        assertEquals("the blocker must be engaged while intaking", Intake.BLOCKER_ENGAGED,
                rig.world.blocker.getPosition(), 1e-6);

        rig.driver.right_trigger = 0.0f;
        rig.run(1.6); // let the flywheel reach the shot speed
        assertTrue("the flywheel should be at the table speed before firing",
                rig.world.shooter.rpm() > 1500);

        rig.driver.right_bumper = true;
        rig.run(0.5);
        System.out.printf("firing: blocker %.3f  rpm %.0f%n", rig.world.blocker.getPosition(), rig.world.shooter.rpm());
        assertEquals("the blocker must release while firing", Intake.BLOCKER_RELEASED,
                rig.world.blocker.getPosition(), 1e-6);
        assertTrue("the intake must keep feeding while firing", rig.world.intakeMotor.getPower() > 0.1);

        rig.driver.right_bumper = false;
        rig.run(0.4);
        assertEquals("letting go of the bumper must close the blocker", Intake.BLOCKER_ENGAGED,
                rig.world.blocker.getPosition(), 1e-6);
        assertEquals("letting go of the bumper must stop the feed", 0.0,
                rig.world.intakeMotor.getPower(), 1e-6);

        rig.opMode.stop();
        System.out.printf("teleop shots %d scored %d%n", rig.world.shotsFired(), rig.world.ballsScored);
        assertTrue("holding fire from the AUTO shooting spot should score: " + rig.world.shotsFired()
                + " fired, " + rig.world.ballsScored + " in", rig.world.ballsScored >= 1);
    }

    /** TELEOP has to start where AUTO left off: same pose, same turret angle, same up CELL. */
    @Test
    public void picksUpWhereAutoLeftOff() throws Exception {
        Pose handover = new Pose(30, 100, Math.toRadians(140));
        Rig rig = new Rig(Alliance.RED, handover, Field.CellSide.FAR, 25.0);

        Pose actual = rig.world.robot.truePose();
        assertEquals("TELEOP must start at AUTO's x", handover.x(), actual.x(), 1.0);
        assertEquals("TELEOP must start at AUTO's y", handover.y(), actual.y(), 1.0);
        assertEquals("TELEOP must start at AUTO's heading", handover.heading(), actual.heading(), 0.05);

        rig.run(0.3);
        String turret = rig.world.telemetry.get("Turret");
        System.out.printf("handover telemetry: %s%n", turret);
        assertTrue("TELEOP must inherit the tipped HIVE's up CELL, got: " + turret,
                turret.contains("FAR CELL up"));
        // the far CELL is on the rear half, and the hand-off pose is too, so firing is legal
        assertTrue("the hand-off pose should be on the CELL's opening side",
                Field.isOnOpeningSide(actual, Field.CellSide.FAR));
        rig.opMode.stop();
    }
}
