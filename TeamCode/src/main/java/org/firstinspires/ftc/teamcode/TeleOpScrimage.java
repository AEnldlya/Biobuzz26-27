package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/** Simple robot-centric drive and three-motor control for scrimmages. */
@TeleOp(name = "TeleOp Scrimage", group = "Biobuzz")
public class TeleOpScrimage extends OpMode {
    private static final double INTAKE_POWER = -1.0;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double SHOOTER_COARSE_STEP = 0.01;
    private static final double SHOOTER_FINE_STEP = 0.001;
    private static final double BLOCKER_INTAKE_POSITION = 0.8;
    private static final double BLOCKER_SHOOT_POSITION = 0.5;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotor shooterMotor;
    private DcMotor shooterMotor2;
    private Servo blockServo;

    private boolean intakeRunning = false;
    private double shooterMagnitude = 0.6;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        // Match the drive motor directions in pedro/Constants.java.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // The flywheels face opposite ways. Keep both at the same signed power.
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        stopMotors();
        blockServo.setPosition(BLOCKER_INTAKE_POSITION);
        telemetry.addLine("Sticks: drive; right trigger: start intake only; left: stop intake");
        telemetry.addLine("Hold A: shoot (both feed motors -1, blocker 0.5)");
        telemetry.addLine("D-pad up/down + right bumper: shooter +/- 0.01; left bumper: +/- 0.001");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double frontLeft = forward + strafe + turn;
        double frontRight = forward - strafe - turn;
        double backLeft = forward - strafe + turn;
        double backRight = forward + strafe - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight))));
        frontLeftMotor.setPower(frontLeft / scale);
        frontRightMotor.setPower(frontRight / scale);
        backLeftMotor.setPower(backLeft / scale);
        backRightMotor.setPower(backRight / scale);

        // Left trigger takes priority if both triggers are pressed.
        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            intakeRunning = false;
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intakeRunning = true;
        }
        boolean shooting = gamepad1.a;
        intakeMotor.setPower(shooting || intakeRunning ? INTAKE_POWER : 0.0);
        transferMotor.setPower(shooting ? INTAKE_POWER : 0.0);
        blockServo.setPosition(shooting ? BLOCKER_SHOOT_POSITION : BLOCKER_INTAKE_POSITION);

        boolean increaseSpeed = gamepad1.dpadUpWasPressed();
        boolean decreaseSpeed = gamepad1.dpadDownWasPressed();
        double speedStep = gamepad1.left_bumper ? SHOOTER_FINE_STEP
                : gamepad1.right_bumper ? SHOOTER_COARSE_STEP : 0.0;
        if (increaseSpeed) shooterMagnitude += speedStep;
        if (decreaseSpeed) shooterMagnitude -= speedStep;
        shooterMagnitude = Math.max(0.0, Math.min(1.0, shooterMagnitude));
        shooterMotor.setPower(-shooterMagnitude);
        shooterMotor2.setPower(-shooterMagnitude);

        telemetry.addData("Feed mode", shooting ? "shoot: -1.0 / -1.0"
                : intakeRunning ? "intake: -1.0 / 0.0" : "stopped");
        telemetry.addData("Blocker position", shooting ? BLOCKER_SHOOT_POSITION : BLOCKER_INTAKE_POSITION);
        telemetry.addData("Shooter power", "%.3f", -shooterMagnitude);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
        transferMotor.setPower(0.0);
        shooterMotor.setPower(0.0);
        shooterMotor2.setPower(0.0);
    }
}
