package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/**
 * Pedro 3 path definitions.
 *
 * Design the BASE route as BLUE + LEFT in the Visualizer.
 * RIGHT reflects that route across the field's vertical center line.
 * RED rotates the resulting route 180 degrees around field center.
 *
 * The transformations assume field symmetry and the coordinate bounds below.
 * Check these assumptions against your season's field and the Visualizer.
 */
public final class Autopaths {

    // IMPORTANT: Use the same coordinate bounds as your Visualizer.
    // Common FTC Pedro coordinates use 0..144 inches on each axis.
    private static final double FIELD_MAX_X = 144.0;
    private static final double FIELD_MAX_Y = 144.0;

    private static final PoseFactory DEGREES = PoseFactory.degrees();

    private AutoPaths() {
        // Utility class; do not instantiate.
    }

    /** One start pose and two consecutive autonomous path stages. */
    public static final class Route {
        public final Pose startPose;
        public final Path toLaunch;
        public final Path toPark;

        public Route(Pose startPose, Path toLaunch, Path toPark) {
            this.startPose = startPose;
            this.toLaunch = toLaunch;
            this.toPark = toPark;
        }
    }

    /**
     * Applies SIDE reflection first, then ALLIANCE rotation.
     * Angle input is in degrees; the resulting Pose stores radians.
     */
    private static Pose pose(
            AutoRoutineConfig config,
            double x,
            double y,
            double headingDegrees) {

        if (config.launchSide == AutoRoutineConfig.LaunchSide.RIGHT) {
            x = FIELD_MAX_X - x;
            headingDegrees = 180.0 - headingDegrees;
        }

        if (config.alliance == AutoRoutineConfig.Alliance.RED) {
            x = FIELD_MAX_X - x;
            y = FIELD_MAX_Y - y;
            headingDegrees += 180.0;
        }

        headingDegrees = ((headingDegrees % 360.0) + 360.0) % 360.0;
        return DEGREES.of(x, y, headingDegrees);
    }

    public static Route build(AutoRoutineConfig config) {

        // ============================================================
        // PASTE VISUALIZER PATH HERE - PART 1: BLUE / LEFT POSES
        // Replace ALL example coordinates with your own Visualizer poses.
        // Keep using pose(config, x, y, headingDegrees) for each one.
        // ============================================================
        Pose start   = pose(config, 24, 24, 0);
        Pose control = pose(config, 35, 40, 45);
        Pose launch  = pose(config, 48, 48, 90);
        Pose park    = pose(config, 60, 48, 90);

        // ============================================================
        // PASTE VISUALIZER PATH HERE - PART 2: BLUE / LEFT PATHS
        // Replace these two sample paths with your exported geometry.
        // Use curve(start, control, launch), line(start, launch), etc.
        // Keep the .linear(...) heading method (or your exported one).
        // ============================================================
        Path toLaunch = curve(start, control, launch)
                .linear(start, launch);

        Path toPark = line(launch, park)
                .linear(launch, park);

        return new Route(start, toLaunch, toPark);
    }
}
