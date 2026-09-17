package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A HardwareMap whose lookups stay in plain Java. The stock get() consults the SDK's Device
 * class, whose static initializer loads the RobotCore native library, which does not exist on
 * a desktop JVM.
 */
public class SimHardwareMap extends HardwareMap {
    private final Map<String, HardwareDevice> devices = new LinkedHashMap<>();

    public SimHardwareMap() {
        super(null, null);
    }

    @Override
    public void put(String deviceName, HardwareDevice device) {
        devices.put(deviceName, device);
        super.put(deviceName, device);
    }

    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        HardwareDevice device = devices.get(deviceName);
        if (device == null) {
            throw new IllegalArgumentException("Unable to find a hardware device with name \"" + deviceName + "\"");
        }
        if (!classOrInterface.isInstance(device)) {
            throw new IllegalArgumentException("Hardware device \"" + deviceName + "\" is not a " + classOrInterface.getSimpleName());
        }
        return classOrInterface.cast(device);
    }

    @Override
    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new ArrayList<>();
        for (HardwareDevice device : devices.values()) {
            if (classOrInterface.isInstance(device)) {
                result.add(classOrInterface.cast(device));
            }
        }
        return result;
    }
}
