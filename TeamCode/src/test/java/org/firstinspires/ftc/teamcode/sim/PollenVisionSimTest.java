package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.vision.Pollen;
import org.firstinspires.ftc.teamcode.vision.PollenVision;
import org.junit.Test;

/** The pollen finder against the virtual Limelight: does it put the pile where the pile is? */
public class PollenVisionSimTest {
    @Test
    public void findsTheBiggestPileOfTheRightColour() throws Exception {
        SimWorld world = new SimWorld(Alliance.RED, 0.0);
        world.robot.setPose(Field.RED_SCAN);
        // pile of four purple 24 in ahead, a lone purple to the side, three green nearer
        double[][] pile = {{20, 68}, {24, 70}, {28, 67}, {23, 73}};
        for (double[] p : pile) {
            world.addPollen(Pollen.PURPLE, p[0], p[1]);
        }
        world.addPollen(Pollen.PURPLE, 8, 62);
        world.addPollen(Pollen.GREEN, 26, 58);
        world.addPollen(Pollen.GREEN, 22, 60);
        world.addPollen(Pollen.GREEN, 24, 56);

        PollenVision vision = new PollenVision(world.camera, world.robot);
        vision.setTarget(Pollen.PURPLE);
        vision.start();
        long start = System.nanoTime();
        while (System.nanoTime() - start < 1_500_000_000L) {
            world.robot.manualDrive(0, 0, 0);
            world.robot.update();
            vision.update();
            Thread.sleep(5);
        }
        vision.stop();

        PollenVision.Cluster best = vision.bestNear(Field.RED_SCAN, Field.MAX_POLLEN_CHASE_IN);
        assertNotNull("no cluster found", best);
        double cx = 0, cy = 0;
        for (double[] p : pile) {
            cx += p[0] / pile.length;
            cy += p[1] / pile.length;
        }
        double off = Math.hypot(best.x - cx, best.y - cy);
        System.out.printf("vision: best cluster at %.1f, %.1f (pile centre %.1f, %.1f, off by %.1f in), weight %.2f, %d clusters, %d frames%n",
                best.x, best.y, cx, cy, off, best.weight, vision.clusters().size(), vision.getFramesUsed());
        assertTrue("best cluster should be the pile of four, off by " + off + " in", off < 5.0);
        Pose approach = PollenVision.approachPose(world.robot.truePose(), best, 14.0, 0.0);
        double approachDist = Math.hypot(approach.x() - best.x, approach.y() - best.y);
        assertTrue("approach pose should stand off the pile", Math.abs(approachDist - 14.0) < 1.0);
        for (PollenVision.Cluster c : vision.clusters()) {
            // nothing green should have leaked in: every cluster must sit on a purple piece
            double nearest = Double.MAX_VALUE;
            for (SimWorld.PollenPiece p : world.pollen) {
                if (p.color == Pollen.PURPLE) {
                    nearest = Math.min(nearest, Math.hypot(p.x - c.x, p.y - c.y));
                }
            }
            assertTrue("cluster not on purple pollen: " + c.x + "," + c.y, nearest < 8.0);
        }
    }
}
