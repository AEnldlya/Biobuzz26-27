package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0 robot config.
 *
 * Drivetrain and Pinpoint values are carried over exactly from the tuned 2.0.4 constants.
 * Foresight is new in 3.0 and has no direct 2.0 equivalent, so its values below are seeded
 * from the old tuning (max velocities, zero-power deceleration, translational/heading P) and
 * MUST be re-tuned with AutoTune: with the robot on, open http://192.168.43.1:10158 and run
 * "3. Foresight" (see Tuning.java).
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // 2.0.4 forwardPodY -> xPodOffset, strafePodX -> yPodOffset (both pass straight to
    // GoBildaPinpointDriver.setOffsets(x, y) in their versions)
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(8.6);
        c.yPodOffset.set(3.86);
        c.offsetUnits.set(DistanceUnit.CM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // 2.0.4: secondary translational P 0.22 under 4 in of error, primary 0.087 beyond
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.22)).put(4.0, Controller.proportional(0.087)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.22)).put(4.0, Controller.proportional(0.087)));

        // power per in/s ~ 1 / max velocity
        c.coast.set(Controller.proportionalFeedforward(1.0 / 54.968));
        c.brake.set(Controller.proportionalFeedforward(1.0 / 54.968));

        c.headingFeedback.set(Controller.proportional(1.0));
        // stopping distance ~ linear*v + quadratic*v|v|; quadratic = 1 / (2 * deceleration)
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.02, 0.04));
        c.linearBrakeCoefficients.set(Matrix.diag(0.02, 0.02));
        c.quadraticBrakeCoefficients.set(Matrix.diag(1.0 / (2 * 68.3), 1.0 / (2 * 79.305)));

        // 2.0.4 xVelocity / yVelocity and |zero power acceleration|
        c.maxAchievableForwardVelocity.set(54.968);
        c.maxAchievableStrafeVelocity.set(44.004);
        c.naturalForwardDeceleration.set(68.3);
        c.naturalStrafeDeceleration.set(79.305);
    });

    public static Localizer localizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Drivetrain drivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Algorithm algorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(localizer(hardwareMap), drivetrain(hardwareMap), algorithm());
    }
}
