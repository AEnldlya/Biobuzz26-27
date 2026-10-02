package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.RedAuto;
import org.junit.Test;

/**
 * The real RED AUTO OpMode, start to park, on the simulated field.
 *
 * The replay lands in TeamCode/build/sim/auto_red.html - open it in a browser.
 */
public class RedAutoSimTest {
    @Test
    public void redAutoTipsCollectsScoresAndParks() throws Exception {
        RedAuto auto = new RedAuto();
        SimWorld world = AutoScenario.run(auto, Alliance.RED, "auto_red");
        AutoScenario.check(world, auto, Alliance.RED);
    }
}
