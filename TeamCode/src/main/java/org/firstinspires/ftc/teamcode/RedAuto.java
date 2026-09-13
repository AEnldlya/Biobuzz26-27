package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "RED AUTO", group = "Biobuzz", preselectTeleOp = "Biobuzz TeleOp")
public class RedAuto extends OpMode {
    private static final Alliance ALLIANCE = Alliance.RED;

    private Hubs hubs;
    private Follower follower;
    private Claw claw;
    private Shooter shooter;
    private Turret turret;
    private final Timer pathTimer = new Timer();
    private final Timer opmodeTimer = new Timer();

    private int pathState = 0;

    // Write every pose for RED; ALLIANCE.fromRed() puts it on this alliance's side.
    private final Pose startPose = Field.startPose(ALLIANCE);
    // private final Pose scorePose = ALLIANCE.fromRed(new Pose(48, 40, Math.toRadians(90)));

    // private Path scorePreload;

    @Override
    public void init() {
        hubs = new Hubs(hardwareMap);
        Battery battery = new Battery(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        claw = new Claw(hardwareMap);
        shooter = new Shooter(hardwareMap, battery);
        hubs.clearCache();
        // the turret must be facing forward at init
        turret = new Turret(hardwareMap, battery);
        turret.setAlliance(ALLIANCE);
        turret.setUpCell(Field.startingUpCell(ALLIANCE));

        buildPaths();
        RobotState.save(ALLIANCE, startPose, turret);
    }

    @Override
    public void init_loop() {
        hubs.clearCache();
        follower.update();
        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        turret.setMode(Turret.Mode.AUTO_AIM);
        shooter.setEnabled(true);
        setPathState(0);
    }

    @Override
    public void loop() {
        hubs.clearCache();
        follower.update();
        Pose pose = follower.pose();

        turret.update(pose, follower.velocity());
        shooter.setShotDistance(turret.getDistance(), 0.0);
        shooter.update();

        autonomousPathUpdate();
        RobotState.save(ALLIANCE, pose, turret);

        telemetry.addData("path state", pathState);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        turret.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        claw.stop();
        claw.close();
        shooter.stop();
        turret.stop();
    }

    private void buildPaths() {
        // Pedro 3.0 examples:
        // scorePreload = line(startPose, scorePose).linear(startPose.heading(), scorePose.heading());
        // Path pickup = curve(scorePose, ALLIANCE.fromRed(new Pose(30, 60)), pickupPose).tangent();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // drive to the preload shot
                break;

            case 1: // fire the preload once turret.isOnTarget() && shooter.atSpeed()
                break;

            case 2: // drive to a pickup
                break;

            case 3: // intake
                break;

            case 4: // drive back to shoot
                break;

            case 5: // fire; if the HIVE tipped, turret.flipUpCell()
                break;

            case 6: // park
                break;

            case 7: // done
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
    }
}
