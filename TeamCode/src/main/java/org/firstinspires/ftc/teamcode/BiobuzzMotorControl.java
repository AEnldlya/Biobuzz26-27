package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/** Simple gamepad 1 controls for the intake, transfer, and shooter motors. */
@TeleOp(name = "Biobuzz Motor Control", group = "Biobuzz")
public class BiobuzzMotorControl extends OpMode {
    private static final double SHOOTER_STEP = 0.1;
    private static final double TRIGGER_DEADZONE = 0.05;
    private static final double BLOCKER_STEP = 0.1;
    private static final double STICK_THRESHOLD = 0.5;
    private static final long BLOCKER_STEP_INTERVAL_NS = 250_000_000L;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotor shooterMotor;
    private Servo blockServo;
    private double feedPower = 0.0;
    private double shooterMagnitude = 0.5;
    private double blockerPosition = 0.45;
    private int blockerStickDirection = 0;
    private long lastBlockerStepNs = 0;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
        shooterMotor.setPower(0.0);
        blockServo.setPosition(blockerPosition);

        telemetry.addLine("Gamepad 1: A forward, B stop, X reverse intake + transfer");
        telemetry.addLine("Right/left trigger: shooter forward/reverse; D-pad right/left: power +/- 0.1");
        telemetry.addLine("Hold right stick up/down: blocker servo +/- 0.1 per step");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Keep the last selected feed direction until another button is pressed.
        if (gamepad1.b) {
            feedPower = 0.0;
        } else if (gamepad1.a) {
            feedPower = 1.0;
        } else if (gamepad1.x) {
            feedPower = -1.0;
        }
        intakeMotor.setPower(feedPower);
        transferMotor.setPower(feedPower);

        if (gamepad1.dpadRightWasPressed()) {
            shooterMagnitude = Math.min(1.0, shooterMagnitude + SHOOTER_STEP);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            shooterMagnitude = Math.max(0.0, shooterMagnitude - SHOOTER_STEP);
        }

        boolean forward = gamepad1.right_trigger > TRIGGER_DEADZONE;
        boolean reverse = gamepad1.left_trigger > TRIGGER_DEADZONE;
        // Releasing both triggers, or pressing both, stops the shooter.
        double shooterPower = forward == reverse ? 0.0 : (forward ? shooterMagnitude : -shooterMagnitude);
        shooterMotor.setPower(shooterPower);

        double rightStickY = gamepad1.right_stick_y;
        // Up is negative on the gamepad axis, so up increases servo position.
        int stickDirection = rightStickY < -STICK_THRESHOLD ? 1
                : rightStickY > STICK_THRESHOLD ? -1 : 0;
        long nowNs = System.nanoTime();
        if (stickDirection != 0 && (stickDirection != blockerStickDirection
                || nowNs - lastBlockerStepNs >= BLOCKER_STEP_INTERVAL_NS)) {
            blockerPosition = Math.max(0.0, Math.min(1.0,
                    blockerPosition + stickDirection * BLOCKER_STEP));
            blockServo.setPosition(blockerPosition);
            lastBlockerStepNs = nowNs;
        }
        blockerStickDirection = stickDirection;

        telemetry.addData("Intake + transfer power", "%.1f", feedPower);
        telemetry.addData("Shooter selected power", "%.1f", shooterMagnitude);
        telemetry.addData("Shooter applied power", "%.1f", shooterPower);
        telemetry.addData("Blocker servo position", "%.2f", blockerPosition);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
        shooterMotor.setPower(0.0);
    }
}
