package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Pedro Pathing 3 localizer for four mecanum DRIVE encoders + REV hub IMU.
 * +X = forward, +Y = robot's left, +heading = counterclockwise.
 * No external odometry pods or camera are required.
 *
 * Calibrate Constants' encoder signs and forward/strafe ticks per inch
 * before using this localizer in a moving autonomous path.
 */
public final class MecanumIMULocalizer implements Localizer {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private int lastFL, lastFR, lastBL, lastBR;
    private double lastYaw;
    private long lastTimeNanos;
    private MotionState state = MotionState.zero();

    public MecanumIMULocalizer(HardwareMap map, MecanumConfig config) {
        frontLeft = map.get(DcMotorEx.class, config.frontLeftName.get());
        frontRight = map.get(DcMotorEx.class, config.frontRightName.get());
        backLeft = map.get(DcMotorEx.class, config.backLeftName.get());
        backRight = map.get(DcMotorEx.class, config.backRightName.get());
        imu = map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(Constants.HUB_ORIENTATION));
        setPose(Pose.zero());
    }

    private double yaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private int flTicks() { return Constants.ENCODER_FL_SIGN * frontLeft.getCurrentPosition(); }
    private int frTicks() { return Constants.ENCODER_FR_SIGN * frontRight.getCurrentPosition(); }
    private int blTicks() { return Constants.ENCODER_BL_SIGN * backLeft.getCurrentPosition(); }
    private int brTicks() { return Constants.ENCODER_BR_SIGN * backRight.getCurrentPosition(); }

    private static double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public void setPose(Pose pose) {
        // Set the field coordinate without resetting motor encoders or hub yaw.
        // This also resets the local measurement baseline.
        lastFL = flTicks();
        lastFR = frTicks();
        lastBL = blTicks();
        lastBR = brTicks();
        lastYaw = yaw();
        lastTimeNanos = System.nanoTime();
        state = MotionState.ofTwist(pose, Twist.zero());
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = (now - lastTimeNanos) / 1e9;
        if (dt <= 0) return;
        lastTimeNanos = now;

        int fl = flTicks(), fr = frTicks(), bl = blTicks(), br = brTicks();
        int dFL = fl - lastFL, dFR = fr - lastFR;
        int dBL = bl - lastBL, dBR = br - lastBR;
        lastFL = fl; lastFR = fr; lastBL = bl; lastBR = br;

        double newYaw = yaw();
        double dHeading = wrapRadians(newYaw - lastYaw);
        lastYaw = newYaw;

        // Pedro Mecanum wheel pattern: positive X forward, positive Y left.
        double forwardTicks = (dFL + dFR + dBL + dBR) / 4.0;
        double leftTicks = (-dFL + dFR + dBL - dBR) / 4.0;
        double forwardIn = forwardTicks / Constants.FORWARD_TICKS_PER_INCH;
        double leftIn = leftTicks / Constants.STRAFE_TICKS_PER_INCH;

        // Pose.exp integrates local forward/strafe movement through the turn.
        Pose nextPose = state.pose().exp(new Twist(forwardIn, leftIn, dHeading));
        double safeDt = Math.max(dt, 1e-4);
        state = MotionState.ofTwist(nextPose,
                new Twist(forwardIn / safeDt, leftIn / safeDt, dHeading / safeDt));
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void reset() {
        setPose(Pose.zero());
    }
}
