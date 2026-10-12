package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Simple gamepad 1 controls for the intake, transfer, and shooter motors. */
@TeleOp(name = "Biobuzz Motor Control", group = "Biobuzz")
public class BiobuzzMotorControl extends OpMode {
    private static final double SHOOTER_STEP = 0.1;
    private static final double TRIGGER_DEADZONE = 0.05;

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotor shooterMotor;
    private double feedPower = 0.0;
    private double shooterMagnitude = 0.5;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
        shooterMotor.setPower(0.0);

        telemetry.addLine("Gamepad 1: A forward, B stop, X reverse intake + transfer");
        telemetry.addLine("Right/left trigger: shooter forward/reverse; D-pad right/left: power +/- 0.1");
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

        telemetry.addData("Intake + transfer power", "%.1f", feedPower);
        telemetry.addData("Shooter selected power", "%.1f", shooterMagnitude);
        telemetry.addData("Shooter applied power", "%.1f", shooterPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
        shooterMotor.setPower(0.0);
    }
}
