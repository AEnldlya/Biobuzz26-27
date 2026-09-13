package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Controls the intake motor and blocker servo. */
public class Claw {
    private static final double SHOT_FEED_POWER = 0.6;

    private final DcMotor intakeMotor;
    private final Servo blockServo;

    public Claw(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void release() {
        blockServo.setPosition(0.25);
    }

    public void close() {
        blockServo.setPosition(0.45);
    }

    public void run() {
        intakeMotor.setPower(1.0);
    }

    public void feedForShot() {
        intakeMotor.setPower(SHOT_FEED_POWER);
    }

    public void stop() {
        intakeMotor.setPower(0.0);
    }
}
