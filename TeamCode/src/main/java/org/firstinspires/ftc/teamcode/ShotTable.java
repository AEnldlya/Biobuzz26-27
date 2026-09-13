package org.firstinspires.ftc.teamcode;

/**
 * Shot settings by distance from the turret pivot to the upward CELL aim point, linearly
 * interpolated and clamped at the ends.
 *
 * STARTING GUESSES ONLY: the BIOBUZZ CELL opening is ~59.5 in high, much higher than the DECODE
 * goal, so none of the old numbers carry over. Fill these in with the Shot Tuner OpMode.
 * Time of flight is only used to lead shots while driving (Turret.LEAD_GAIN).
 */
public final class ShotTable {
    public static double[] DISTANCE_IN = {24, 48, 72, 96, 120, 144};
    public static double[] RPM = {3400, 3800, 4300, 4800, 5300, 5800};
    public static double[] HOOD = {0.20, 0.30, 0.40, 0.48, 0.56, 0.64};
    public static double[] TIME_OF_FLIGHT_S = {0.28, 0.38, 0.48, 0.56, 0.62, 0.68};

    private ShotTable() {
    }

    public static double rpm(double distanceIn) {
        return interpolate(RPM, distanceIn);
    }

    public static double hood(double distanceIn) {
        return interpolate(HOOD, distanceIn);
    }

    public static double timeOfFlight(double distanceIn) {
        return interpolate(TIME_OF_FLIGHT_S, distanceIn);
    }

    private static double interpolate(double[] values, double distanceIn) {
        double[] distances = DISTANCE_IN;
        int last = distances.length - 1;
        if (Double.isNaN(distanceIn) || distanceIn <= distances[0]) {
            return values[0];
        }
        if (distanceIn >= distances[last]) {
            return values[last];
        }
        for (int i = 1; i <= last; i++) {
            if (distanceIn <= distances[i]) {
                double t = (distanceIn - distances[i - 1]) / (distances[i] - distances[i - 1]);
                return values[i - 1] + t * (values[i] - values[i - 1]);
            }
        }
        return values[last];
    }
}
