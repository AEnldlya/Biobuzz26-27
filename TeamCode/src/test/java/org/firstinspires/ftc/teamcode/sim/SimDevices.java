package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.DoubleSupplier;

/**
 * Fake FTC hardware devices. They only remember what the robot code writes and hand back what
 * the physics models say; the models live in SimWorld and friends.
 */
public final class SimDevices {
    private SimDevices() {
    }

    /** A servo with signed power (continuous rotation). */
    public static class SimCRServo implements CRServo {
        private final String name;
        private double power = 0.0;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        public SimCRServo(String name) {
            this.name = name;
        }

        /** power as the mechanism sees it (direction applied) */
        public double effectivePower() {
            return direction == DcMotorSimple.Direction.REVERSE ? -power : power;
        }

        @Override public ServoController getController() { return null; }
        @Override public int getPortNumber() { return 0; }
        @Override public void setDirection(DcMotorSimple.Direction d) { direction = d; }
        @Override public DcMotorSimple.Direction getDirection() { return direction; }
        @Override public void setPower(double p) { power = Math.max(-1, Math.min(1, p)); }
        @Override public double getPower() { return power; }
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return name; }
        @Override public String getConnectionInfo() { return "sim"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    /** A positional servo. */
    public static class SimServo implements Servo {
        private final String name;
        private double position = 0.0;
        private Servo.Direction direction = Servo.Direction.FORWARD;

        public SimServo(String name) {
            this.name = name;
        }

        @Override public ServoController getController() { return null; }
        @Override public int getPortNumber() { return 0; }
        @Override public void setDirection(Servo.Direction d) { direction = d; }
        @Override public Servo.Direction getDirection() { return direction; }
        @Override public void setPosition(double p) { position = Math.max(0, Math.min(1, p)); }
        @Override public double getPosition() { return position; }
        @Override public void scaleRange(double min, double max) { }
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return name; }
        @Override public String getConnectionInfo() { return "sim"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    /** A motor whose encoder position and velocity come from a model. */
    public static class SimMotor implements DcMotorEx {
        private final String name;
        private double power = 0.0;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        private DcMotor.ZeroPowerBehavior zeroPower = DcMotor.ZeroPowerBehavior.FLOAT;
        /** ticks per second the model reports; direction is applied on the way out */
        public DoubleSupplier ticksPerSecond = () -> 0.0;
        public DoubleSupplier ticks = () -> 0.0;

        public SimMotor(String name) {
            this.name = name;
        }

        /** power with direction applied, i.e. what the mechanism feels */
        public double effectivePower() {
            return direction == DcMotorSimple.Direction.REVERSE ? -power : power;
        }

        private double sign() {
            return direction == DcMotorSimple.Direction.REVERSE ? -1.0 : 1.0;
        }

        @Override public void setMotorEnable() { }
        @Override public void setMotorDisable() { }
        @Override public boolean isMotorEnabled() { return true; }
        @Override public void setVelocity(double v) { }
        @Override public void setVelocity(double v, AngleUnit unit) { }
        @Override public double getVelocity() { return sign() * ticksPerSecond.getAsDouble(); }
        @Override public double getVelocity(AngleUnit unit) { return getVelocity(); }
        @Override public void setPIDCoefficients(DcMotor.RunMode m, PIDCoefficients c) { }
        @Override public void setPIDFCoefficients(DcMotor.RunMode m, PIDFCoefficients c) { }
        @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { }
        @Override public void setPositionPIDFCoefficients(double p) { }
        @Override public PIDCoefficients getPIDCoefficients(DcMotor.RunMode m) { return null; }
        @Override public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode m) { return null; }
        @Override public void setTargetPositionTolerance(int t) { }
        @Override public int getTargetPositionTolerance() { return 0; }
        @Override public double getCurrent(CurrentUnit unit) { return 0; }
        @Override public double getCurrentAlert(CurrentUnit unit) { return 0; }
        @Override public void setCurrentAlert(double c, CurrentUnit unit) { }
        @Override public boolean isOverCurrent() { return false; }
        @Override public MotorConfigurationType getMotorType() { return null; }
        @Override public void setMotorType(MotorConfigurationType t) { }
        @Override public DcMotorController getController() { return null; }
        @Override public int getPortNumber() { return 0; }
        @Override public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior z) { zeroPower = z; }
        @Override public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() { return zeroPower; }
        @Override public void setPowerFloat() { }
        @Override public boolean getPowerFloat() { return false; }
        @Override public void setTargetPosition(int p) { }
        @Override public int getTargetPosition() { return 0; }
        @Override public boolean isBusy() { return false; }
        @Override public int getCurrentPosition() { return (int) Math.round(sign() * ticks.getAsDouble()); }
        @Override public void setMode(DcMotor.RunMode m) { mode = m; }
        @Override public DcMotor.RunMode getMode() { return mode; }
        @Override public void setDirection(DcMotorSimple.Direction d) { direction = d; }
        @Override public DcMotorSimple.Direction getDirection() { return direction; }
        @Override public void setPower(double p) { power = Math.max(-1, Math.min(1, p)); }
        @Override public double getPower() { return power; }
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return name; }
        @Override public String getConnectionInfo() { return "sim"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    /** Analog input channels whose voltages come from suppliers. */
    public static class SimAnalogController implements AnalogInputController {
        private final DoubleSupplier[] channels;

        public SimAnalogController(DoubleSupplier... channels) {
            this.channels = channels;
        }

        @Override public double getAnalogInputVoltage(int channel) {
            return Math.max(0.0, Math.min(3.3, channels[channel].getAsDouble()));
        }
        @Override public double getMaxAnalogInputVoltage() { return 3.3; }
        @Override public SerialNumber getSerialNumber() { return null; }
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return "sim analog"; }
        @Override public String getConnectionInfo() { return "sim"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    public static class SimVoltageSensor implements VoltageSensor {
        public double volts = 12.8;

        @Override public double getVoltage() { return volts; }
        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return "sim battery"; }
        @Override public String getConnectionInfo() { return "sim"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }

    /** Anything else the code looks up but the sim doesn't model. */
    public static class SimPlainDevice implements HardwareDevice {
        private final String name;

        public SimPlainDevice(String name) {
            this.name = name;
        }

        @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
        @Override public String getDeviceName() { return name; }
        @Override public String getConnectionInfo() { return "sim"; }
        @Override public int getVersion() { return 1; }
        @Override public void resetDeviceConfigurationForOpMode() { }
        @Override public void close() { }
    }
}
