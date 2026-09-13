package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Manual bulk caching: all encoder and velocity reads in a loop come from one bulk read per hub
 * instead of one hub transaction each. Call clearCache() once at the top of every loop.
 */
public class Hubs {
    private final List<LynxModule> hubs;

    public Hubs(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void clearCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
