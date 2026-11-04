package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/**
 * Pedro 3 setup. Merge calibrated MecanumTuner and ForesightTuner output here.
 * The default Foresight values only let the project compile and test manually;
 * they are NOT measured and MUST NOT be used for field autonomous.
 */
public final class Constants {
    private Constants() {}

    // Backward-compatible names for the DriveEncoderCheck/localizer from earlier ZIP.
    public static final RevHubOrientationOnRobot HUB_ORIENTATION = RobotConfig.HUB_ORIENTATION;
    public static int ENCODER_FL_SIGN = RobotConfig.ENCODER_FL_SIGN;
    public static int ENCODER_FR_SIGN = RobotConfig.ENCODER_FR_SIGN;
    public static int ENCODER_BL_SIGN = RobotConfig.ENCODER_BL_SIGN;
    public static int ENCODER_BR_SIGN = RobotConfig.ENCODER_BR_SIGN;
    public static double FORWARD_TICKS_PER_INCH = RobotConfig.FORWARD_TICKS_PER_INCH;
    public static double STRAFE_TICKS_PER_INCH = RobotConfig.STRAFE_TICKS_PER_INCH;

    // REPLACE ONLY THIS BLOCK with Mecanum AutoTune output when ready.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.FRONT_LEFT);
        c.frontRightName.set(RobotConfig.FRONT_RIGHT);
        c.backLeftName.set(RobotConfig.BACK_LEFT);
        c.backRightName.set(RobotConfig.BACK_RIGHT);
        c.frontLeftDirection.set(RobotConfig.FL_DIRECTION);
        c.frontRightDirection.set(RobotConfig.FR_DIRECTION);
        c.backLeftDirection.set(RobotConfig.BL_DIRECTION);
        c.backRightDirection.set(RobotConfig.BR_DIRECTION);
    });

    // UNVERIFIED INITIAL VALUES. REPLACE THIS BLOCK with Foresight AutoTune output.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.proportional(1.0));
        c.forwardTranslational.set(Controller.proportional(0.045));
        c.strafeTranslational.set(Controller.proportional(0.045));
        c.coast.set(Controller.proportionalFeedforward(0.01));
        c.brake.set(Controller.proportionalFeedforward(0.08));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06, 0.01));
        c.linearBrakeCoefficients.set(Matrix.diag(0.12, 0.12));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.015, 0.015));
        c.maxAchievableForwardVelocity.set(20.0);
        c.maxAchievableStrafeVelocity.set(15.0);
        c.naturalForwardDeceleration.set(20.0);
        c.naturalStrafeDeceleration.set(15.0);
        c.maxPathSpeed.set(0.25);  // Low power until actual tuning
    });

    public static Follower create(HardwareMap map) {
        return create(map, new Mecanum(map, drivetrainConfig));
    }

    /** Reuses Robot.drive so only one driver issues power to the same motors. */
    public static Follower create(HardwareMap map, Mecanum drive) {
        return new Follower(
                new MecanumIMULocalizer(map, drivetrainConfig),
                drive,
                new Foresight(foresightConfig));
    }
}
