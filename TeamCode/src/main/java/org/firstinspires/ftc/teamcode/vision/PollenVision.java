package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.pedro.PoseHistory;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Limelight colour-blob pollen finder for AUTO: "go to where there is the most of a certain
 * pollen".
 *
 * The Limelight runs a colour pipeline per pollen colour (see Pollen) and reports every blob it
 * sees as a ColorResult with a horizontal / vertical angle and an image area. Each loop this
 * class:
 * <ol>
 *   <li>takes the newest frame once (frames are ~30-90 Hz, the loop can be faster),</li>
 *   <li>projects every blob onto the floor from the camera mount geometry (height, pitch, and
 *       the known height of a pollen piece), giving a point in the robot frame,</li>
 *   <li>moves that point into the field frame using the robot pose FROM WHEN THE FRAME WAS
 *       CAPTURED (a poseAt() callback backed by the localizer's pose history), so results stay
 *       right while the robot is turning to scan,</li>
 *   <li>merges the point into a persistent field map of clusters: a blob within CLUSTER_RADIUS_IN
 *       of an existing cluster joins it (weighted average position, weight += blob area), else
 *       it starts a new one. Cluster weights decay every second so pollen that has been
 *       picked up or pushed away fades out.</li>
 * </ol>
 * best() is the heaviest cluster, i.e. the most pollen of the selected colour. approachPose()
 * turns it into a Pedro pose to drive to with the intake facing it.
 *
 * Units: inches and radians in the field frame; the Limelight gives degrees (tx positive =
 * right of the crosshair, ty positive = above it) and area as a percentage of the image.
 */
public class PollenVision {
    public static String NAME = "limelight";
    public static int POLL_RATE_HZ = 60;

    // ---- camera mount, all relative to the robot's odometry tracking centre ----
    public static double CAMERA_FORWARD_IN = 5.5;   // + = toward the robot's front (12 in robot)
    public static double CAMERA_LEFT_IN = 0.0;      // + = toward the robot's left
    public static double CAMERA_HEIGHT_IN = 11.0;   // lens centre above the tiles
    public static double CAMERA_PITCH_DEG = 22.0;   // tilt DOWN from horizontal (+ = down)
    public static double CAMERA_YAW_DEG = 0.0;      // + = camera turned to the left of forward
    /** height of the point on a pollen piece the blob centre corresponds to (~its centre) */
    public static double POLLEN_HEIGHT_IN = 2.5;

    // ---- blob filtering ----
    public static double MIN_BLOB_AREA = 0.03;      // % of image; drops specks
    public static double MAX_RANGE_IN = 84.0;       // beyond this the floor projection is junk
    public static double MIN_RANGE_IN = 4.0;
    public static long MAX_STALENESS_MS = 250;      // ignore frames older than this
    public static double FIELD_MARGIN_IN = 4.0;     // clusters must be this far inside the walls

    // ---- clustering / memory ----
    public static double CLUSTER_RADIUS_IN = 10.0;
    /** fraction of a cluster's weight that survives each second it is not seen */
    public static double DECAY_PER_S = 0.5;
    public static double MIN_CLUSTER_WEIGHT = 0.05;
    public static int MAX_CLUSTERS = 12;
    /** ignore a cluster as "best" until it has been seen in at least this many frames */
    public static int MIN_SIGHTINGS = 2;

    /** Somewhere on the field with pollen of the selected colour. */
    public static final class Cluster {
        public double x;
        public double y;
        /** accumulated, decayed blob area: the amount of pollen there */
        public double weight;
        public int sightings;
        public long lastSeenNs;

        public Pose toPose() {
            return new Pose(x, y);
        }
    }

    private final PollenCamera camera;
    private final PoseHistory poseHistory;
    private final List<Cluster> clusters = new ArrayList<>();

    private Pollen target = Pollen.PURPLE;
    private boolean running = false;
    private double lastFrameTimestamp = -1.0;
    private long lastFrameNs = 0;
    private long lastDecayNs = 0;
    private int lastBlobCount = 0;
    private int lastUsedCount = 0;
    private int framesUsed = 0;
    private int reportedPipeline = -1;

    /** How the hardware-map constructor finds its camera; the simulator swaps in a virtual one. */
    public static Function<HardwareMap, PollenCamera> CAMERA_FACTORY =
            hardwareMap -> new LimelightPollenCamera(hardwareMap, NAME);

    /** The real robot: a Limelight named NAME. */
    public PollenVision(HardwareMap hardwareMap, PoseHistory poseHistory) {
        this(CAMERA_FACTORY.apply(hardwareMap), poseHistory);
    }

    /** Any camera, e.g. the simulator's. */
    public PollenVision(PollenCamera camera, PoseHistory poseHistory) {
        this.camera = camera;
        this.poseHistory = poseHistory;
    }

    /** Start polling the camera on the selected pollen's pipeline. */
    public void start() {
        if (!camera.isConnected()) {
            return;
        }
        camera.start(target.pipeline, POLL_RATE_HZ);
        running = true;
        lastDecayNs = System.nanoTime();
    }

    public void stop() {
        if (camera.isConnected() && running) {
            camera.stop();
        }
        running = false;
    }

    /** Which pollen colour to look for; switching pipelines clears the map. */
    public void setTarget(Pollen pollen) {
        if (pollen == target) {
            return;
        }
        target = pollen;
        clusters.clear();
        lastFrameTimestamp = -1.0;
        if (camera.isConnected() && running) {
            camera.switchPipeline(pollen.pipeline);
        }
    }

    public Pollen getTarget() {
        return target;
    }

    public void clear() {
        clusters.clear();
    }

    /** Call every loop after the localizer has updated. */
    public void update() {
        long now = System.nanoTime();
        decay(now);
        if (!camera.isConnected() || !running) {
            return;
        }

        PollenCamera.Frame frame = camera.latest();
        if (frame == null) {
            return;
        }
        reportedPipeline = frame.pipelineIndex;
        if (reportedPipeline != target.pipeline) {
            // still switching pipelines: those blobs are the wrong colour
            return;
        }
        if (frame.timestamp == lastFrameTimestamp) {
            return; // already used this frame
        }
        if (frame.stalenessMs > MAX_STALENESS_MS) {
            return;
        }
        lastFrameTimestamp = frame.timestamp;
        lastFrameNs = now;
        framesUsed++;

        // pose when the image was captured: now, minus how long ago the result arrived
        // (staleness, wall clock ms), minus the camera's own capture + processing latency
        long captureNs = now - frame.stalenessMs * 1_000_000L - (long) (frame.latencyMs * 1e6);
        Pose robot = poseHistory.poseAt(captureNs);

        lastBlobCount = frame.blobs.size();
        lastUsedCount = 0;
        for (PollenCamera.Blob blob : frame.blobs) {
            if (blob.area < MIN_BLOB_AREA) {
                continue;
            }
            Pose point = project(robot, blob.txDeg, blob.tyDeg);
            if (point == null) {
                continue;
            }
            merge(point.x(), point.y(), blob.area, now);
            lastUsedCount++;
        }
        prune();
    }

    /**
     * Floor intersection of the ray through a blob, in the field frame, or null when the ray
     * does not hit the floor at a usable range.
     */
    Pose project(Pose robot, double txDeg, double tyDeg) {
        double below = Math.toRadians(CAMERA_PITCH_DEG - tyDeg);   // angle of the ray below horizontal
        if (below < Math.toRadians(1.0)) {
            return null; // at or above the horizon
        }
        double drop = CAMERA_HEIGHT_IN - POLLEN_HEIGHT_IN;
        if (drop <= 0) {
            return null;
        }
        double range = drop / Math.tan(below);           // along the floor, from the lens
        if (range < MIN_RANGE_IN || range > MAX_RANGE_IN) {
            return null;
        }
        // bearing in the robot frame: camera yaw (left +) minus tx (right +)
        double bearing = Math.toRadians(CAMERA_YAW_DEG - txDeg);
        double forward = CAMERA_FORWARD_IN + range * Math.cos(bearing);
        double left = CAMERA_LEFT_IN + range * Math.sin(bearing);

        double heading = robot.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double x = robot.x() + forward * cos - left * sin;
        double y = robot.y() + forward * sin + left * cos;
        if (x < FIELD_MARGIN_IN || x > Field.SIZE - FIELD_MARGIN_IN
                || y < FIELD_MARGIN_IN || y > Field.SIZE - FIELD_MARGIN_IN) {
            return null;
        }
        return new Pose(x, y);
    }

    private void merge(double x, double y, double area, long now) {
        Cluster nearest = null;
        double nearestDist = CLUSTER_RADIUS_IN;
        for (Cluster c : clusters) {
            double d = Math.hypot(c.x - x, c.y - y);
            if (d < nearestDist) {
                nearestDist = d;
                nearest = c;
            }
        }
        if (nearest == null) {
            Cluster c = new Cluster();
            c.x = x;
            c.y = y;
            c.weight = area;
            c.sightings = 1;
            c.lastSeenNs = now;
            clusters.add(c);
            return;
        }
        double total = nearest.weight + area;
        nearest.x = (nearest.x * nearest.weight + x * area) / total;
        nearest.y = (nearest.y * nearest.weight + y * area) / total;
        nearest.weight = total;
        if (nearest.lastSeenNs != now) {
            nearest.sightings++;
            nearest.lastSeenNs = now;
        }
    }

    private void decay(long now) {
        if (lastDecayNs == 0) {
            lastDecayNs = now;
            return;
        }
        double seconds = (now - lastDecayNs) / 1e9;
        lastDecayNs = now;
        if (seconds <= 0 || clusters.isEmpty()) {
            return;
        }
        double factor = Math.pow(DECAY_PER_S, seconds);
        for (Cluster c : clusters) {
            c.weight *= factor;
        }
        prune();
    }

    private void prune() {
        for (int i = clusters.size() - 1; i >= 0; i--) {
            if (clusters.get(i).weight < MIN_CLUSTER_WEIGHT) {
                clusters.remove(i);
            }
        }
        while (clusters.size() > MAX_CLUSTERS) {
            int lightest = 0;
            for (int i = 1; i < clusters.size(); i++) {
                if (clusters.get(i).weight < clusters.get(lightest).weight) {
                    lightest = i;
                }
            }
            clusters.remove(lightest);
        }
    }

    /** The cluster with the most pollen, or null if nothing has been seen. */
    public Cluster best() {
        Cluster best = null;
        for (Cluster c : clusters) {
            if (c.sightings < MIN_SIGHTINGS) {
                continue;
            }
            if (best == null || c.weight > best.weight) {
                best = c;
            }
        }
        return best;
    }

    /** Heaviest cluster within maxDistanceIn of a point (e.g. the robot), or null. */
    public Cluster bestNear(Pose from, double maxDistanceIn) {
        Cluster best = null;
        for (Cluster c : clusters) {
            if (c.sightings < MIN_SIGHTINGS || Math.hypot(c.x - from.x(), c.y - from.y()) > maxDistanceIn) {
                continue;
            }
            if (best == null || c.weight > best.weight) {
                best = c;
            }
        }
        return best;
    }

    public List<Cluster> clusters() {
        return clusters;
    }

    /**
     * Where to drive so the intake is standoffIn short of the cluster and facing it, coming in
     * along the line from the robot's current position. intakeHeadingOffset is 0 for an intake
     * on the robot's front, Math.PI for one on the back.
     */
    public static Pose approachPose(Pose robot, Cluster cluster, double standoffIn, double intakeHeadingOffset) {
        double dx = cluster.x - robot.x();
        double dy = cluster.y - robot.y();
        double dist = Math.hypot(dx, dy);
        double bearing = dist < 1e-6 ? robot.heading() : Math.atan2(dy, dx);
        double back = Math.min(standoffIn, Math.max(dist - 1.0, 0.0));
        double x = cluster.x - back * Math.cos(bearing);
        double y = cluster.y - back * Math.sin(bearing);
        return new Pose(x, y, Angle.normalize(bearing + intakeHeadingOffset));
    }

    /** A point pushThroughIn past the cluster along the approach direction (drive through it). */
    public static Pose throughPose(Pose approach, Cluster cluster, double pushThroughIn, double intakeHeadingOffset) {
        double bearing = Angle.normalize(approach.heading() - intakeHeadingOffset);
        double x = cluster.x + pushThroughIn * Math.cos(bearing);
        double y = cluster.y + pushThroughIn * Math.sin(bearing);
        x = Math.max(FIELD_MARGIN_IN, Math.min(Field.SIZE - FIELD_MARGIN_IN, x));
        y = Math.max(FIELD_MARGIN_IN, Math.min(Field.SIZE - FIELD_MARGIN_IN, y));
        return new Pose(x, y, approach.heading());
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public boolean isRunning() {
        return running;
    }

    /** true if a frame was used within the last MAX_STALENESS_MS */
    public boolean hasFreshFrame() {
        return lastFrameNs != 0 && System.nanoTime() - lastFrameNs < MAX_STALENESS_MS * 1_000_000L;
    }

    public int getLastBlobCount() {
        return lastBlobCount;
    }

    public int getFramesUsed() {
        return framesUsed;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (!camera.isConnected()) {
            telemetry.addData("Pollen", "NO LIMELIGHT in configuration (\"%s\")", NAME);
            return;
        }
        telemetry.addData("Pollen", "%s pipeline %d (camera says %d) %s", target, target.pipeline,
                reportedPipeline, hasFreshFrame() ? "" : "STALE");
        telemetry.addData("Pollen blobs", "%d seen, %d on floor, %d frames, %d clusters",
                lastBlobCount, lastUsedCount, framesUsed, clusters.size());
        Cluster best = best();
        if (best != null) {
            telemetry.addData("Pollen best", "x %.1f  y %.1f  weight %.2f  seen %d",
                    best.x, best.y, best.weight, best.sightings);
        }
    }
}
