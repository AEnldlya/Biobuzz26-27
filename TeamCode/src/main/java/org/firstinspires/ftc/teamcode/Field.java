package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * BIOBUZZ field geometry in Pedro coordinates (inches, radians), from Competition Manual V1
 * section 9.
 *
 * Looking at the field from the audience: (0, 0) is the bottom-left corner, the red ALLIANCE
 * wall is x = 0 and the audience wall is y = 0. Heading 0 faces +x (toward the blue wall) and
 * increases counter-clockwise.
 *
 * The HIVE structure sits at field center. Each alliance's HIVE tips about a pivot so one of its
 * two CELLS faces up; that upward CELL is what the turret aims at.
 */
public final class Field {
    public static final double SIZE = 144.0;
    public static final double CENTER = SIZE / 2.0;

    /** Which end of a HIVE has its CELL tipped up. */
    public enum CellSide {
        AUDIENCE,
        FAR;

        public CellSide flipped() {
            return this == AUDIENCE ? FAR : AUDIENCE;
        }
    }

    // HIVE centerlines are 25.5 in apart (Figure 9-10); the red HIVE is on the red side
    public static double HIVE_HALF_SPACING = 25.5 / 2.0;
    // aim point distance from the pivot line toward the upward CELL; tipped 30 deg, the CELL
    // center is ~13.4 in out and its opening face ~18.6 in out
    public static double CELL_AIM_OFFSET = 15.0;
    // the upward CELL opening spans 53.5 to 65.6 in above the TILES
    public static final double CELL_OPENING_HEIGHT_IN = 59.5;

    // PLACEHOLDER: measure your real red start. G304: on the red half, touching a wall.
    public static Pose RED_START = new Pose(48, 9, Math.toRadians(90));

    // ---- AUTO waypoints, written for RED; Alliance.fromRed() mirrors them for BLUE ----
    // PLACEHOLDERS until the field is measured. All must be on the opening side of the
    // starting up-CELL (red starts with the AUDIENCE cell up, so y < CENTER while shooting).
    /** where the robot stops to shoot; the turret aims itself, heading just has to be sane */
    public static Pose RED_SCORE = new Pose(40, 40, Math.toRadians(90));
    /** where the robot stops and looks for pollen with the Limelight */
    public static Pose RED_SCAN = new Pose(24, 44, Math.toRadians(90));
    /** used when the camera sees nothing: drive here and intake anyway */
    public static Pose RED_PICKUP_FALLBACK = new Pose(24, 60, Math.toRadians(90));
    public static Pose RED_PARK = new Pose(14, 14, Math.toRadians(90));

    /** furthest from RED_SCAN the auto will chase pollen (keeps it out of the other alliance) */
    public static double MAX_POLLEN_CHASE_IN = 48.0;

    private Field() {
    }

    /** Figure 10-2: red starts with its audience-side CELL up, blue with its far-side CELL up. */
    public static CellSide startingUpCell(Alliance alliance) {
        return alliance == Alliance.RED ? CellSide.AUDIENCE : CellSide.FAR;
    }

    public static Pose startPose(Alliance alliance) {
        return alliance.fromRed(RED_START);
    }

    public static Pose cellAimPoint(Alliance alliance, CellSide upCell) {
        double x = alliance == Alliance.RED ? CENTER - HIVE_HALF_SPACING : CENTER + HIVE_HALF_SPACING;
        double y = upCell == CellSide.FAR ? CENTER + CELL_AIM_OFFSET : CENTER - CELL_AIM_OFFSET;
        return new Pose(x, y);
    }

    /**
     * The upward CELL opens away from the HIVE center, so shots only go in from that side.
     * Launching into the outside of a CELL is a G417 violation.
     */
    public static boolean isOnOpeningSide(Pose robot, CellSide upCell) {
        return upCell == CellSide.FAR ? robot.y() > CENTER : robot.y() < CENTER;
    }
}
