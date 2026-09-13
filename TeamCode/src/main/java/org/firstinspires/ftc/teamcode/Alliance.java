package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * The BIOBUZZ field is point-symmetric about its center, so every blue position is the
 * matching red position rotated 180 degrees. Author poses for red and convert with fromRed().
 */
public enum Alliance {
    RED,
    BLUE;

    public Pose fromRed(Pose redPose) {
        if (this == RED) {
            return redPose;
        }
        return new Pose(Field.SIZE - redPose.x(), Field.SIZE - redPose.y(), redPose.heading() + Math.PI);
    }
}
