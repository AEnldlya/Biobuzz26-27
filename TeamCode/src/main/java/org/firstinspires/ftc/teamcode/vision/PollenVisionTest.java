package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.Hubs;
import org.firstinspires.ftc.teamcode.RobotState;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FusedPinpointLocalizer;

/**
 * Checks the pollen finder before trusting it in AUTO.
 *
 * Put one pollen piece at a measured spot, drive around it, and compare "Pollen best" with
 * where it really is (the field origin is the robot's start pose, gamepad 1 X / B pick the
 * alliance). If the range is wrong adjust CAMERA_HEIGHT_IN / CAMERA_PITCH_DEG; if it is
 * mirrored left-right check CAMERA_YAW_DEG and the tx sign; if it smears while turning the
 * latency compensation or the mount offsets are off.
 *
 * Gamepad 1
 *   sticks: drive           dpad up/down: next / previous pollen colour
 *   A: clear the map        Y: reset pose to the start pose
 */
@TeleOp(name = "Pollen Vision Test", group = "Tuning")
public class PollenVisionTest extends OpMode {
    private Hubs hubs;
    private Follower follower;
    private FusedPinpointLocalizer localizer;
    private PollenVision vision;
    private GamePiece[] pollens = GamePiece.values();
    private int pollenIndex = 0;
    private Alliance alliance;

    @Override
    public void init() {
        hubs = new Hubs(hardwareMap);
        follower = Constants.create(hardwareMap);
        localizer = Constants.fusedLocalizer(follower);
        alliance = RobotState.alliance;
        follower.setPose(Field.startPose(alliance));
        vision = new PollenVision(hardwareMap, Constants.poseHistory(follower));
        vision.setTarget(pollens[pollenIndex]);
    }

    @Override
    public void init_loop() {
        hubs.clearCache();
        // the field frame is anchored to the alliance start pose, so a cluster's reported
        // coordinates only mean anything once this matches the wall the robot is against
        if (gamepad1.xWasPressed()) {
            alliance = Alliance.BLUE;
            follower.setPose(Field.startPose(alliance));
        }
        if (gamepad1.bWasPressed()) {
            alliance = Alliance.RED;
            follower.setPose(Field.startPose(alliance));
        }
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("Alliance", "%s   (gamepad 1: X blue, B red)", alliance);
        telemetry.addData("Limelight", vision.isConnected() ? "found, " + pollens[pollenIndex] : "NOT FOUND");
        telemetry.addData("Start pose", "put the robot on the %s wall start spot: x %.1f  y %.1f",
                alliance, pose.x(), pose.y());
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        hubs.clearCache();
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
        vision.update();

        if (gamepad1.dpadUpWasPressed()) {
            pollenIndex = (pollenIndex + 1) % pollens.length;
            vision.setTarget(pollens[pollenIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            pollenIndex = (pollenIndex + pollens.length - 1) % pollens.length;
            vision.setTarget(pollens[pollenIndex]);
        }
        if (gamepad1.aWasPressed()) {
            vision.clear();
        }
        if (gamepad1.yWasPressed()) {
            follower.setPose(Field.startPose(alliance));
        }

        Pose pose = follower.pose();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        if (localizer != null) {
            telemetry.addData("Pinpoint", "%s%s", localizer.status(), localizer.usingImuFallback() ? "  HUB IMU FALLBACK" : "");
        }
        vision.addTelemetry(telemetry);
        int shown = 0;
        for (PollenVision.Cluster c : vision.clusters()) {
            if (shown++ >= 5) {
                break;
            }
            telemetry.addData("Cluster " + shown, "x %.1f  y %.1f  w %.2f  seen %d", c.x, c.y, c.weight, c.sightings);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}
