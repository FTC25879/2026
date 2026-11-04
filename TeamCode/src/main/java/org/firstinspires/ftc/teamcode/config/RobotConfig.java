package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * EDIT THIS FILE FIRST. Hardware names must match Driver Station Configuration.
 * Motor/servo settings are cautious placeholders, NOT verified on Team 25879 hardware.
 */
public final class RobotConfig {
    private RobotConfig() {}

    public static final String FRONT_LEFT = "frontLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_LEFT = "backLeft";
    public static final String BACK_RIGHT = "backRight";
    public static final String INTAKE = "intake";  // One motor driving both intake rollers
    public static final String SHOOTER = "shooter"; // Expansion Hub motor port 4
    public static final String GATE = "gate";       // Confirm your servo config
    public static final String STICK = "stick";     // Expansion Hub servo port 5
    public static final String IMU = "imu";

    // Confirm directions on the REAL robot. MecanumTuner outputs the correct values.
    public static final DcMotorSimple.Direction FL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Change these to the actual Control Hub mounting orientation.
    public static final RevHubOrientationOnRobot HUB_ORIENTATION =
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

    // Encoder signs: +1/-1 chosen so all four readings increase while driving forward.
    // These are independent from motor directions and must be calibrated.
    public static final int ENCODER_FL_SIGN = 1;
    public static final int ENCODER_FR_SIGN = 1;
    public static final int ENCODER_BL_SIGN = 1;
    public static final int ENCODER_BR_SIGN = 1;
    public static final double FORWARD_TICKS_PER_INCH = 100.0; // PLACEHOLDER
    public static final double STRAFE_TICKS_PER_INCH = 100.0;  // PLACEHOLDER
    public static final boolean LOCALIZATION_CALIBRATED = false; // Set true AFTER physical tests

    // Servo travel is deliberately small at first. Calibrate with linkage clear.
    public static final double GATE_CLOSED = 0.50;
    public static final double GATE_OPEN = 0.57;
    public static final double STICK_RETRACTED = 0.50;
    public static final double STICK_EXTENDED = 0.60;
    public static final double GATE_OPEN_SECONDS = 0.20;
    public static final double GATE_SETTLE_SECONDS = 0.20;

    public static final double INTAKE_POWER = 0.65;

    // CRITICAL: Set ticks per *physical flywheel output revolution*, accounting for gearing.
    // The example 28 is only for a one-to-one 28-count output. Verify hardware first.
    public static final double SHOOTER_TICKS_PER_REV = 28.0; // MUST VERIFY
    public static final double SHOOTER_INITIAL_RPM = 180.0;  // MUST CALIBRATE
    public static final double SHOOTER_MAX_RPM = 6000.0;     // SOFTWARE UPPER BOUND, NOT MOTOR CAPABILITY
    public static final double SHOOTER_STEP_RPM = 10.0;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 12.0;
    public static final double SHOOTER_READY_STABLE_SECONDS = 0.30;
    public static final double SHOOTER_MAX_AMPS = 5.0;       // Team's reported protection limit
    public static final double SHOOTER_OVER_CURRENT_SECONDS = 0.25;
    public static final double MIN_BATTERY_VOLTAGE = 10.5;   // Stop shooter below this
    public static final double SHOT_READY_TIMEOUT_SECONDS = 3.0;

    // Limit automated testing. These are not verified BIOBUZZ field coordinates.
    public static final double AUTO_TEST_DRIVE_POWER = 0.22;
    public static final double AUTO_TEST_DRIVE_SECONDS = 0.75;
    public static final double AUTO_TEST_FORWARD_INCHES = 12.0;
    public static final double AUTO_PATH_TIMEOUT_SECONDS = 3.5;
    public static final int AUTO_MAX_SHOTS = 4;

    // Hard interlocks: leave false until drive localization/paths and shooting are validated.
    public static final boolean ENABLE_PEDRO_AUTONOMOUS = false;
    public static final boolean ENABLE_AUTO_SHOOTING = false;
}
