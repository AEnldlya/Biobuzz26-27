package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Hand-off from AUTO to TELEOP. Static fields survive between OpModes as long as the Robot
 * Controller app keeps running, so TELEOP starts where AUTO left the robot, turret, and HIVE.
 */
public final class RobotState {
    public static Alliance alliance = Alliance.RED;
    /** null when nothing has been saved since the app started */
    public static Pose pose = null;
    public static double turretAngleDeg = 0.0;
    public static Field.CellSide upCell = Field.startingUpCell(Alliance.RED);

    private RobotState() {
    }

    public static void save(Alliance alliance, Pose pose, Turret turret) {
        RobotState.alliance = alliance;
        RobotState.pose = pose;
        RobotState.turretAngleDeg = turret.getAngleDeg();
        RobotState.upCell = turret.getUpCell();
    }

    public static void clearPose() {
        pose = null;
    }
}
