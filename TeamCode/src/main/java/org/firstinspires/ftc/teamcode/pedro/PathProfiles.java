package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Per-path Foresight overrides ("custom path constraints" in Pedro 3.0).
 *
 * Foresight decides a path is finished when translational error, heading error and speed are
 * all under its constraints, or the timeout expires. Its defaults are 0.1 in, 0.4 deg, 0.1 in/s
 * and a 100 ms timeout, which no real robot settles to, so every path either stalls at its end
 * or gets cut off by the timeout. Constants sets sane defaults for the whole robot; these
 * profiles override them per path with Path.with(...). The overrides are applied when the path
 * starts and reverted when it ends (PathTracker does that), so they never leak into the next
 * path.
 *
 *   follower.follow(PathProfiles.straight(from, to, PathProfiles.score()));
 *
 * All distances in inches, angles in radians, times in milliseconds, maxPathSpeed is a
 * fraction of the max achievable velocity.
 */
public final class PathProfiles {
    // stop precisely: the turret and shooter are about to fire from here
    public static double SCORE_TRANSLATIONAL_IN = 0.75;
    public static double SCORE_HEADING_RAD = Math.toRadians(1.5);
    public static double SCORE_VELOCITY_IN_S = 2.0;
    public static double SCORE_TIMEOUT_MS = 900;

    // get there fast; the next path or the pollen seeker corrects the rest
    public static double TRANSIT_TRANSLATIONAL_IN = 2.0;
    public static double TRANSIT_HEADING_RAD = Math.toRadians(4.0);
    public static double TRANSIT_VELOCITY_IN_S = 6.0;
    public static double TRANSIT_TIMEOUT_MS = 500;

    // drive through pollen slowly enough for the intake to grab it
    public static double PICKUP_SPEED = 0.6;
    public static double PICKUP_TRANSLATIONAL_IN = 2.0;
    public static double PICKUP_HEADING_RAD = Math.toRadians(5.0);
    public static double PICKUP_VELOCITY_IN_S = 8.0;
    public static double PICKUP_TIMEOUT_MS = 400;

    public static double PARK_SPEED = 0.85;
    public static double PARK_TRANSLATIONAL_IN = 3.0;
    public static double PARK_HEADING_RAD = Math.toRadians(6.0);
    public static double PARK_VELOCITY_IN_S = 10.0;
    public static double PARK_TIMEOUT_MS = 300;

    private PathProfiles() {
    }

    /**
     * A straight path from one pose to another that really ENDS at the second pose's heading.
     *
     * Pedro 3.0.0's Path.linear(a, b) holds b at t = 0 and a at t = 1, i.e. its arguments read
     * backwards from their names, so a plain linear(from.heading(), to.heading()) drives the
     * robot to the heading it STARTED at. That is invisible on a turret robot until the camera
     * or the intake has to point somewhere. Everything that builds a path goes through here so
     * there is one place to change if a later Pedro release swaps them back; PathHeadingTest in
     * the simulator pins the behaviour down and fails loudly if it does.
     */
    public static Path straight(Pose from, Pose to, Modifier... profile) {
        return Paths.line(from, to).linear(to.heading(), from.heading()).with(profile);
    }

    public static Modifier[] score() {
        return new Modifier[] {
                Constants.foresightConfig.translationalConstraint.at(SCORE_TRANSLATIONAL_IN),
                Constants.foresightConfig.headingConstraint.at(SCORE_HEADING_RAD),
                Constants.foresightConfig.velocityConstraint.at(SCORE_VELOCITY_IN_S),
                Constants.foresightConfig.timeoutConstraint.at(SCORE_TIMEOUT_MS),
                Constants.foresightConfig.brakeAtEnd.at(true),
        };
    }

    public static Modifier[] transit() {
        return new Modifier[] {
                Constants.foresightConfig.translationalConstraint.at(TRANSIT_TRANSLATIONAL_IN),
                Constants.foresightConfig.headingConstraint.at(TRANSIT_HEADING_RAD),
                Constants.foresightConfig.velocityConstraint.at(TRANSIT_VELOCITY_IN_S),
                Constants.foresightConfig.timeoutConstraint.at(TRANSIT_TIMEOUT_MS),
        };
    }

    public static Modifier[] pickup() {
        return new Modifier[] {
                Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED),
                Constants.foresightConfig.translationalConstraint.at(PICKUP_TRANSLATIONAL_IN),
                Constants.foresightConfig.headingConstraint.at(PICKUP_HEADING_RAD),
                Constants.foresightConfig.velocityConstraint.at(PICKUP_VELOCITY_IN_S),
                Constants.foresightConfig.timeoutConstraint.at(PICKUP_TIMEOUT_MS),
                Constants.foresightConfig.brakeAtEnd.at(false),
        };
    }

    public static Modifier[] park() {
        return new Modifier[] {
                Constants.foresightConfig.maxPathSpeed.at(PARK_SPEED),
                Constants.foresightConfig.translationalConstraint.at(PARK_TRANSLATIONAL_IN),
                Constants.foresightConfig.headingConstraint.at(PARK_HEADING_RAD),
                Constants.foresightConfig.velocityConstraint.at(PARK_VELOCITY_IN_S),
                Constants.foresightConfig.timeoutConstraint.at(PARK_TIMEOUT_MS),
        };
    }
}
