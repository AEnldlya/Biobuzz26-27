package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake motor + transfer motor + blocker servo.
 *
 * The intake and transfer motors always run together: on while intaking and while shooting
 * (shooting is intaking straight through into the flywheel), off otherwise. The blocker sits
 * between the transfer and the flywheel: engaged (blocking) while intaking so balls stack up
 * behind it, released while shooting so the transfer feeds them through.
 *
 *   OFF     motors off,  blocker engaged
 *   INTAKE  motors on,   blocker engaged
 *   SHOOT   motors on,   blocker released
 */
public class Intake {
    public static String INTAKE_MOTOR_NAME = "intakeMotor";
    public static String TRANSFER_MOTOR_NAME = "transferMotor";
    public static String BLOCKER_SERVO_NAME = "blockServo";

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    /** feed speed while shooting: slow enough that the flywheel recovers between balls */
    public static double SHOOT_INTAKE_POWER = 0.6;
    public static double SHOOT_TRANSFER_POWER = 0.6;
    public static double BLOCKER_ENGAGED = 0.45;
    public static double BLOCKER_RELEASED = 0.25;

    public enum Mode {
        OFF,
        INTAKE,
        SHOOT
    }

    private final DcMotor intakeMotor;
    private final DcMotor transferMotor;
    private final Servo blocker;
    private Mode mode = null;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        transferMotor = hardwareMap.get(DcMotor.class, TRANSFER_MOTOR_NAME);
        blocker = hardwareMap.get(Servo.class, BLOCKER_SERVO_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(Mode.OFF);
    }

    public void setMode(Mode next) {
        if (next == mode) {
            return; // hardware writes are hub transactions, skip repeats
        }
        mode = next;
        switch (next) {
            case INTAKE:
                intakeMotor.setPower(INTAKE_POWER);
                transferMotor.setPower(TRANSFER_POWER);
                blocker.setPosition(BLOCKER_ENGAGED);
                break;
            case SHOOT:
                intakeMotor.setPower(SHOOT_INTAKE_POWER);
                transferMotor.setPower(SHOOT_TRANSFER_POWER);
                blocker.setPosition(BLOCKER_RELEASED);
                break;
            default:
                intakeMotor.setPower(0.0);
                transferMotor.setPower(0.0);
                blocker.setPosition(BLOCKER_ENGAGED);
                break;
        }
    }

    public Mode getMode() {
        return mode;
    }

    public void intake() {
        setMode(Mode.INTAKE);
    }

    public void shoot() {
        setMode(Mode.SHOOT);
    }

    public void stop() {
        setMode(Mode.OFF);
    }

    public boolean isIntaking() {
        return mode == Mode.INTAKE;
    }

    public boolean isShooting() {
        return mode == Mode.SHOOT;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", "%s  blocker %s", mode, mode == Mode.SHOOT ? "released" : "engaged");
    }
}
