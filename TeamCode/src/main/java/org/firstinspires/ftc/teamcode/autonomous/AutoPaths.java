package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.config.RobotConfig;


/**
 * Four competition start poses are deliberately UNCALIBRATED.
 * Fill in measured field x, y (inches), heading (radians) before setting
 * RobotConfig.AUTO_FIELD_POSES_VERIFIED = true.
 * Until then only robot-relative drive TEST routines are supported.
 */
public final class AutoPaths {
    private AutoPaths() {}

    // Placeholder. DO NOT treat these as field coordinates.
    public static final Pose BLUE_AUDIENCE_START = Pose.zero();
    public static final Pose BLUE_OPPOSITE_START = Pose.zero();
    public static final Pose RED_AUDIENCE_START = Pose.zero();
    public static final Pose RED_OPPOSITE_START = Pose.zero();

    public static Pose selectedStart(AutoRoutineConfig cfg) {
        if (cfg.alliance == AutoRoutineConfig.Alliance.BLUE)
            return cfg.side == AutoRoutineConfig.StartSide.AUDIENCE
                ? BLUE_AUDIENCE_START : BLUE_OPPOSITE_START;
        return cfg.side == AutoRoutineConfig.StartSide.AUDIENCE
            ? RED_AUDIENCE_START : RED_OPPOSITE_START;
    }

    public static Pose testStart() { return Pose.zero(); }
    public static Path testForward() {
        return line(Pose.zero(), new Pose(RobotConfig.AUTO_TEST_FORWARD_INCHES, 0, 0)).constant(0);
    }

    /** Robot-relative test. Not a BIOBUZZ park, target, or mirrored alliance path. */
    public static Path selectedFieldTest(AutoRoutineConfig cfg) {
        if (!RobotConfig.AUTO_FIELD_POSES_VERIFIED)
            throw new IllegalStateException("Field starting poses have not been verified");
        Pose start = selectedStart(cfg);
        double d = RobotConfig.AUTO_TEST_FORWARD_INCHES;
        Pose end = new Pose(start.x() + d * Math.cos(start.heading()),
                            start.y() + d * Math.sin(start.heading()), start.heading());
        return line(start, end).constant(start.heading());
    }
}
