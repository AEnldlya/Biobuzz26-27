package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * BIOBUZZ (2026-2027) field geometry in Pedro coordinates, from the Competition Manual V1
 * (TU01) Section 9 ARENA and Section 10 Game Details, and the Event Field Setup Guide V1.0.
 *
 * Frames. The manual's official frame has its origin at field centre, +Y from the red wall
 * toward the blue wall, +X toward the audience. Pedro here uses inches from the corner: the
 * red ALLIANCE wall is x = 0, the blue wall x = 144, the audience wall y = 0, the rear wall
 * y = 144. So pedro_x = Y + 72 and pedro_y = 72 - X. Heading 0 faces +x (toward the blue
 * wall) and increases counter-clockwise. Everything is written for RED and mirrored for BLUE
 * by Alliance.fromRed() (the field is point-symmetric about its centre).
 *
 * Key published numbers (inches unless noted, general tolerance +/- 1 in):
 *   FIELD 144 x 144 on 24 in TILES.  POLLEN 2.8 in / 24.9 g yellow, 40 per match; NECTAR
 *   3.62 in / 41.3 g red or blue, 8 per alliance.  HIVE pivots 43.95 above the TILES, 25.5
 *   apart (red at official Y = -12.75), arm tilted 30 deg; each CELL mouth 20 wide x 14 tall x
 *   12 deep with its lip 53.5 and top 65.6 above the TILES when up.  A HIVE tips with 8 POLLEN
 *   or 3 POLLEN + 3 NECTAR (setup guide calibration); 3 NECTAR are staged in each up CELL, so
 *   3 launched POLLEN tip it.  LOADING ZONE 23 x 11 against the alliance wall, rear half;
 *   GARDEN a 23 x 2 tape strip along the audience (red) / rear (blue) wall from the alliance
 *   corner, with 4 POLLEN in a line.  Robots start touching their alliance wall with 4 POLLEN,
 *   inside an 18 in cube.  Points: LEAVE 3, PARK (LOADING ZONE) 5, HIVE TIP 20, element left
 *   in the up CELL 2, GARDEN element 1.  Possession limit 4 (G407).
 */
public final class Field {
    public static final double SIZE = 144.0;
    public static final double CENTER = SIZE / 2.0;
    public static final double TILE = 24.0;

    // ---- scoring elements (9.8) ----
    public static final double POLLEN_DIAMETER_IN = 2.8;
    public static final double POLLEN_MASS_KG = 0.0249;
    public static final double NECTAR_DIAMETER_IN = 3.62;
    public static final double NECTAR_MASS_KG = 0.0413;
    /** one NECTAR weighs this many POLLEN */
    public static final double NECTAR_POLLEN_EQUIVALENT = NECTAR_MASS_KG / POLLEN_MASS_KG;
    /** POLLEN-equivalents in the up CELL that tip the HIVE (8 POLLEN, or 3 NECTAR + 3 POLLEN) */
    public static double TIP_POLLEN_EQUIVALENTS = 7.9;
    public static final int NECTAR_STAGED_IN_UP_CELL = 3;
    public static final int PRELOAD_POLLEN = 4;
    public static final int POSSESSION_LIMIT = 4;
    /** how long a HIVE takes to swing over once it lets go (community measurement) */
    public static double HIVE_TIP_TIME_S = 0.88;

    // ---- robot ----
    /** the robot is a 12 x 12 in square; odometry centre at its middle */
    public static double ROBOT_SIZE_IN = 12.0;
    public static double ROBOT_HALF_IN = ROBOT_SIZE_IN / 2.0;
    public static final double START_CUBE_IN = 18.0;

    // ---- HIVE structure (9.6, 9.7) ----
    public static final double HIVE_PIVOT_HEIGHT_IN = 43.95;
    public static final double HIVE_HALF_SPACING = 25.5 / 2.0;
    public static final double HIVE_TILT_DEG = 30.0;
    /** the open mouth of a CELL sits this far from the pivot along the arm */
    public static final double CELL_MOUTH_ALONG_ARM_IN = 21.46;
    public static final double CELL_MOUTH_WIDTH_IN = 20.0;
    public static final double CELL_MOUTH_HEIGHT_IN = 14.0;
    public static final double CELL_DEPTH_IN = 12.0;
    public static final double CELL_LIP_HEIGHT_IN = 53.5;
    public static final double CELL_TOP_HEIGHT_IN = 65.6;
    /**
     * Horizontal distance from the pivot line to the up CELL's mouth centre, taken as the arm
     * length projected at the 30 deg tilt. The manual publishes the mouth HEIGHTS (lip 53.5,
     * top 65.6, so a 12.1 in vertical span = 14 in of mouth at 60 deg from horizontal, which
     * is what fixes the mouth plane's orientation) but not the horizontal offset, and the real
     * CELL is offset from the arm centreline, so this one number is derived rather than
     * published. It only moves the aim point a couple of inches; measure it on a real field.
     */
    public static final double CELL_AIM_OFFSET = CELL_MOUTH_ALONG_ARM_IN * Math.cos(Math.toRadians(HIVE_TILT_DEG));
    /** height of the up CELL's mouth centre, midway between the published lip and top */
    public static final double CELL_OPENING_HEIGHT_IN = (CELL_LIP_HEIGHT_IN + CELL_TOP_HEIGHT_IN) / 2.0;
    /** frame footprint: 49.46 across the field (our x), 38.95 audience-to-rear (our y) */
    public static final double HIVE_FRAME_X_IN = 49.46;
    public static final double HIVE_FRAME_Y_IN = 38.95;

    /** Which end of a HIVE has its CELL tipped up. */
    public enum CellSide {
        AUDIENCE,
        FAR;

        public CellSide flipped() {
            return this == AUDIENCE ? FAR : AUDIENCE;
        }
    }

    /** An axis-aligned rectangle on the tiles. */
    public static final class Rect {
        public final double x0, y0, x1, y1;

        public Rect(double x0, double y0, double x1, double y1) {
            this.x0 = Math.min(x0, x1);
            this.y0 = Math.min(y0, y1);
            this.x1 = Math.max(x0, x1);
            this.y1 = Math.max(y0, y1);
        }

        public boolean contains(double x, double y) {
            return x >= x0 && x <= x1 && y >= y0 && y <= y1;
        }

        /** does a square of half-size h centred at (x, y) overlap this rectangle */
        public boolean overlapsSquare(double x, double y, double h) {
            return x + h >= x0 && x - h <= x1 && y + h >= y0 && y - h <= y1;
        }

        public Rect mirrored() {
            return new Rect(SIZE - x1, SIZE - y1, SIZE - x0, SIZE - y0);
        }
    }

    // ---- zones (9.3, 9.4), red; blue is the point mirror ----
    /** official X in [-48, -24], Y in [-72, -61]: against the red wall, rear half */
    public static final Rect RED_LOADING_ZONE = new Rect(0, 96, 11, 120);
    /** official X in [68, 70], Y in [-72, -48]: a tape strip along the audience wall from the red corner */
    public static final Rect RED_GARDEN = new Rect(0, 2, 24, 4);
    /** the 4 GARDEN POLLEN: a line from the corner, touching the audience wall (setup guide 11.4) */
    public static final double[][] RED_GARDEN_POLLEN = {{1.4, 1.4}, {4.2, 1.4}, {7.0, 1.4}, {9.8, 1.4}};
    /** FLOWER centres on the red side: red wall audience side, rear wall red side (official (24,-69.7), (-69.7,-24)) */
    public static final double[][] RED_FLOWERS = {{2.3, 48.0}, {48.0, 141.7}};
    public static final double FLOWER_TOP_HEIGHT_IN = 21.5;

    // ---- AUTO waypoints, written for RED ----
    /** touching the red wall (x = half size), audience half, clear of the LOADING ZONE and FLOWER, facing the field */
    public static Pose RED_START = new Pose(ROBOT_HALF_IN, 30, 0.0);
    /**
     * Shooting spot for the CELL that is up at the START of the match, about 32 in out from its
     * mouth on the side it opens toward, clear of the HIVE frame. Written for RED (whose start
     * CELL is the AUDIENCE one, so this is on the audience half); fromRed() mirrors it for BLUE,
     * whose start CELL is the FAR one, landing it correctly on the rear half.
     */
    public static Pose RED_SCORE_START_CELL = new Pose(34, 34, 0.0);
    /** shooting spot for the OTHER CELL, the one that comes up after the first HIVE TIP */
    public static Pose RED_SCORE_FLIPPED_CELL = new Pose(34, 104, 0.0);
    /** look at the red GARDEN line from here (heading toward the corner) */
    public static Pose RED_SCAN = new Pose(24, 16, Math.toRadians(218));
    /** if the camera sees nothing: nose into the corner and intake along the wall */
    public static Pose RED_PICKUP_FALLBACK = new Pose(12, 8, Math.toRadians(218));
    /** PARK: at least partially inside the LOADING ZONE */
    public static Pose RED_PARK = new Pose(7, 108, 0.0);
    /** furthest from RED_SCAN the auto will chase POLLEN (keeps it out of the other alliance's half) */
    public static double MAX_POLLEN_CHASE_IN = 36.0;

    private Field() {
    }

    // ---- frames ----

    /** official manual coordinates (inches, X toward the audience, Y toward blue) to Pedro */
    public static Pose fromOfficial(double xOfficial, double yOfficial, double headingOfficialRad) {
        return new Pose(yOfficial + CENTER, CENTER - xOfficial, headingOfficialRad - Math.PI / 2.0);
    }

    // ---- per alliance ----

    /** Figure 10-2: red starts with its audience-side CELL up, blue with its far-side CELL up. */
    public static CellSide startingUpCell(Alliance alliance) {
        return alliance == Alliance.RED ? CellSide.AUDIENCE : CellSide.FAR;
    }

    public static Pose startPose(Alliance alliance) {
        return alliance.fromRed(RED_START);
    }

    /**
     * Where to shoot the currently-up CELL from. Keyed on whether that CELL is the one this
     * alliance STARTED with, not on AUDIENCE / FAR: those are absolute (the audience wall is
     * y = 0), so fromRed() mirrors a spot from one side of the HIVE to the other, and the two
     * alliances start with opposite CELLS up. The result is always on the half the up CELL
     * opens toward, which isOnOpeningSide() requires (G417).
     */
    public static Pose scorePose(Alliance alliance, CellSide upCell) {
        boolean startCell = upCell == startingUpCell(alliance);
        return alliance.fromRed(startCell ? RED_SCORE_START_CELL : RED_SCORE_FLIPPED_CELL);
    }

    public static Pose scanPose(Alliance alliance) {
        return alliance.fromRed(RED_SCAN);
    }

    public static Pose fallbackPickupPose(Alliance alliance) {
        return alliance.fromRed(RED_PICKUP_FALLBACK);
    }

    public static Pose parkPose(Alliance alliance) {
        return alliance.fromRed(RED_PARK);
    }

    public static Rect loadingZone(Alliance alliance) {
        return alliance == Alliance.RED ? RED_LOADING_ZONE : RED_LOADING_ZONE.mirrored();
    }

    public static Rect garden(Alliance alliance) {
        return alliance == Alliance.RED ? RED_GARDEN : RED_GARDEN.mirrored();
    }

    /** PARK: the robot footprint overlaps the alliance LOADING ZONE */
    public static boolean isParked(Alliance alliance, Pose robot) {
        return loadingZone(alliance).overlapsSquare(robot.x(), robot.y(), ROBOT_HALF_IN);
    }

    /** LEAVE: no longer contacting the alliance wall */
    public static boolean hasLeft(Alliance alliance, Pose robot) {
        double gap = alliance == Alliance.RED ? robot.x() - ROBOT_HALF_IN : SIZE - robot.x() - ROBOT_HALF_IN;
        return gap > 0.5;
    }

    // ---- the CELL mouth ----

    public static double hiveX(Alliance alliance) {
        return alliance == Alliance.RED ? CENTER - HIVE_HALF_SPACING : CENTER + HIVE_HALF_SPACING;
    }

    /** the up CELL's mouth centre on the tiles plane (what the turret aims at) */
    public static Pose cellAimPoint(Alliance alliance, CellSide upCell) {
        double y = upCell == CellSide.FAR ? CENTER + CELL_AIM_OFFSET : CENTER - CELL_AIM_OFFSET;
        return new Pose(hiveX(alliance), y);
    }

    /**
     * The open mouth of the up CELL: a 20 x 14 rectangle in a plane perpendicular to the tilted
     * arm. Its normal points out along the arm, 30 deg above horizontal toward the audience
     * (AUDIENCE cell) or the rear (FAR cell); a ball scores when it crosses the plane against
     * the normal inside the rectangle.
     */
    public static final class CellMouth {
        public final double cx, cy, cz;
        /** unit normal, out of the mouth (toward the shooter) */
        public final double[] normal;
        /** unit vector across the mouth (our +x) */
        public final double[] across = {1.0, 0.0, 0.0};
        /** unit vector "up" within the mouth plane */
        public final double[] up;
        public final double halfWidth = CELL_MOUTH_WIDTH_IN / 2.0;
        public final double halfHeight = CELL_MOUTH_HEIGHT_IN / 2.0;

        CellMouth(double cx, double cy, double cz, double[] normal, double[] up) {
            this.cx = cx;
            this.cy = cy;
            this.cz = cz;
            this.normal = normal;
            this.up = up;
        }

        /** signed distance of a point from the plane, positive on the outside */
        public double side(double x, double y, double z) {
            return normal[0] * (x - cx) + normal[1] * (y - cy) + normal[2] * (z - cz);
        }

        /** is a point (assumed on the plane) inside the mouth, for a ball of the given radius */
        public boolean inside(double x, double y, double z, double ballRadius) {
            double w = across[0] * (x - cx) + across[1] * (y - cy) + across[2] * (z - cz);
            double b = up[0] * (x - cx) + up[1] * (y - cy) + up[2] * (z - cz);
            return Math.abs(w) <= halfWidth - ballRadius && Math.abs(b) <= halfHeight - ballRadius;
        }
    }

    public static CellMouth cellMouth(Alliance alliance, CellSide upCell) {
        double tilt = Math.toRadians(HIVE_TILT_DEG);
        Pose aim = cellAimPoint(alliance, upCell);
        double out = upCell == CellSide.FAR ? 1.0 : -1.0; // +y is toward the rear
        double[] normal = {0.0, out * Math.cos(tilt), Math.sin(tilt)};
        double[] up = {0.0, -out * Math.sin(tilt), Math.cos(tilt)};
        return new CellMouth(aim.x(), aim.y(), CELL_OPENING_HEIGHT_IN, normal, up);
    }

    /**
     * The upward CELL opens away from the HIVE center, so shots only go in from that side.
     * Launching into the outside of a CELL is a G417 violation.
     */
    public static boolean isOnOpeningSide(Pose robot, CellSide upCell) {
        return upCell == CellSide.FAR ? robot.y() > CENTER : robot.y() < CENTER;
    }

    /** clamp a robot centre so the 12 in footprint stays inside the walls with an inch to spare */
    public static double clampRobotX(double x) {
        return Math.max(ROBOT_HALF_IN + 1.0, Math.min(SIZE - ROBOT_HALF_IN - 1.0, x));
    }

    public static double clampRobotY(double y) {
        return Math.max(ROBOT_HALF_IN + 1.0, Math.min(SIZE - ROBOT_HALF_IN - 1.0, y));
    }
}
