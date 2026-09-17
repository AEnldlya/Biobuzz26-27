package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.vision.GamePiece;
import org.firstinspires.ftc.teamcode.vision.PollenVision;
import org.junit.Test;

/** The pollen finder against the virtual Limelight: does it put the GARDEN line where it is? */
public class PollenVisionSimTest {
    @Test
    public void findsTheGardenPollenAndIgnoresNectar() throws Exception {
        SimWorld world = new SimWorld(Alliance.RED, 0.0).stagePerRules();
        world.robot.setPose(Field.RED_SCAN);
        // opponent NECTAR nearer the camera than the GARDEN: wrong colour, must not show up
        world.addPiece(GamePiece.BLUE_NECTAR, 14, 8);
        world.addPiece(GamePiece.BLUE_NECTAR, 17, 6);

        PollenVision vision = new PollenVision(world.camera, world.robot);
        vision.setTarget(GamePiece.POLLEN);
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
        for (double[] p : Field.RED_GARDEN_POLLEN) {
            cx += p[0] / Field.RED_GARDEN_POLLEN.length;
            cy += p[1] / Field.RED_GARDEN_POLLEN.length;
        }
        double off = Math.hypot(best.x - cx, best.y - cy);
        System.out.printf("vision: best cluster at %.1f, %.1f (GARDEN line centre %.1f, %.1f, off by %.1f in), weight %.2f, %d clusters, %d frames%n",
                best.x, best.y, cx, cy, off, best.weight, vision.clusters().size(), vision.getFramesUsed());
        assertTrue("best cluster should be the GARDEN line, off by " + off + " in", off < 5.0);
        Pose approach = PollenVision.approachPose(world.robot.truePose(), best, 14.0, 0.0);
        assertTrue("approach pose must keep the robot inside the walls: " + approach,
                approach.x() >= Field.ROBOT_HALF_IN && approach.y() >= Field.ROBOT_HALF_IN);
        for (PollenVision.Cluster c : vision.clusters()) {
            double nearestPollen = Double.MAX_VALUE;
            for (SimWorld.Piece p : world.pieces) {
                if (p.type == GamePiece.POLLEN) {
                    nearestPollen = Math.min(nearestPollen, Math.hypot(p.x - c.x, p.y - c.y));
                }
            }
            assertTrue("cluster not on POLLEN (NECTAR leaked in?): " + c.x + "," + c.y, nearestPollen < 8.0);
        }
    }
}
