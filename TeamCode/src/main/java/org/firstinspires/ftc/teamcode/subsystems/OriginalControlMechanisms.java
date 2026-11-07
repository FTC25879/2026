package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Manual mechanisms using the EXACT proven OnMecanumDrive A/X/Y bindings.
 * Shooter: G1 A held = power 1, release = 0.
 * Ball pusher: G1 X held = power 1, release = 0.
 * Servo0: G1 Y rising edge = 0.50; falling edge = 0.189.
 * Intake was NOT in the working example. G2 triggers provide an optional,
 * manually configurable intake motor test. No shooter RPM or shot sequencer
 * is allowed to control the same devices at the same time.
 */
public final class OriginalControlMechanisms {
    private static final double SERVO_START = 0.189;
    private static final double SERVO_REVERSE = 0.50;

    // No intake existed in OnMecanumDrive. Update this name to your Robot Configuration.
    private static final String INTAKE_NAME = "intake";
    // Conservative initial TEST power. Increase only after verifying actual direction.
    private static final double INTAKE_TEST_POWER = 0.50;

    private final DcMotor shooter;
    private final DcMotor ballPusher;
    private final Servo servo0;
    private final DcMotor intake; // optional until the intake hardware name is confirmed
    private boolean previousY = false;
    private double intakeCommand = 0.0;

    public OriginalControlMechanisms(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        ballPusher = hardwareMap.get(DcMotor.class, "ballPusher");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        intake = hardwareMap.tryGet(DcMotor.class, INTAKE_NAME);

        // Proven shooter motor direction. Do not change ballPusher or servo direction.
        shooter.setDirection(DcMotor.Direction.REVERSE);
    }

    public void update(Gamepad driver, Gamepad operator) {
        // Identical to original OnMecanumDrive for the three proven mechanisms.
        shooter.setPower(driver.a ? 1.0 : 0.0);
        ballPusher.setPower(driver.x ? 1.0 : 0.0);

        boolean currentY = driver.y;
        if (currentY && !previousY) {
            servo0.setPosition(SERVO_REVERSE);
        }
        if (!currentY && previousY) {
            servo0.setPosition(SERVO_START);
        }
        previousY = currentY;

        // Intake is a new mapping, not a claim about the original Android code.
        // It intentionally uses G2 so G1 triggers remain reserved for driving.
        intakeCommand = 0.0;
        if (intake != null) {
            if (operator.right_trigger > 0.2 && operator.left_trigger <= 0.2) {
                intakeCommand = INTAKE_TEST_POWER;
            } else if (operator.left_trigger > 0.2 && operator.right_trigger <= 0.2) {
                intakeCommand = -INTAKE_TEST_POWER;
            }
            intake.setPower(intakeCommand);
        }
    }

    public boolean hasIntake() { return intake != null; }
    public String intakeName() { return INTAKE_NAME; }
    public double intakePower() { return intakeCommand; }
    public double servoCommand() { return servo0.getPosition(); }

    /** Stop motor power and return the momentary servo to the known start command. */
    public void stop() {
        shooter.setPower(0.0);
        ballPusher.setPower(0.0);
        if (intake != null) intake.setPower(0.0);
        intakeCommand = 0.0;
        if (previousY) {
            servo0.setPosition(SERVO_START);
            previousY = false;
        }
    }
}
