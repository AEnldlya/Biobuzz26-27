package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/** Supplies the robot pose at an earlier System.nanoTime() instant (camera latency compensation). */
public interface PoseHistory {
    Pose poseAt(long nanoTime);
}
