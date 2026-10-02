package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.BlueAuto;
import org.junit.Test;

/**
 * The real BLUE AUTO OpMode. Everything is mirrored from RED, and the two alliances start
 * with opposite CELLS up, which is where the mirroring has historically gone wrong.
 *
 * The replay lands in TeamCode/build/sim/auto_blue.html - open it in a browser.
 */
public class BlueAutoSimTest {
    @Test
    public void blueAutoTipsCollectsScoresAndParks() throws Exception {
        BlueAuto auto = new BlueAuto();
        SimWorld world = AutoScenario.run(auto, Alliance.BLUE, "auto_blue");
        AutoScenario.check(world, auto, Alliance.BLUE);
    }
}
