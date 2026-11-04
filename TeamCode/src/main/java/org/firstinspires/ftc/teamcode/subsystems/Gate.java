package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Positional servo: gate in front of the flywheel. */
public final class Gate {
    private final Servo servo;
    private boolean open;

    public Gate(HardwareMap map) {
        servo = map.get(Servo.class, RobotConfig.GATE);
        close();
    }
    public void open() { servo.setPosition(RobotConfig.GATE_OPEN); open = true; }
    public void close() { servo.setPosition(RobotConfig.GATE_CLOSED); open = false; }
    public boolean isOpen() { return open; }
}



/**
package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Android Gate")
public class Gate extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    private DcMotorEx shooter;
    private DcMotor ballPusher;

    private Servo servo0;

    // =================================
    // SHOOTER SETTINGS
    // =================================

    private static final double SHOOTER_TARGET_RPM = 160.0;

    // BARE REV HD HEX MOTOR
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // =================================
    // SERVO POSITIONS
    // =================================

    private static final double SERVO_START = 0.189;
    private static final double SERVO_REVERSE = 0.50;

    // =================================
    // DRIVE SPEED SETTINGS
    // =================================

    private static final double NORMAL_SPEED = 1.0;
    private static final double PRECISION_SPEED = 0.25;

    // =================================
    // BALL COUNTER
    // =================================

    private int ballCount = 0;

    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousY = false;

    @Override
    public void runOpMode() {

        double turn;
        double forward;
        double strafe;
        double pow1;
        double pow2;

        // =================================
        // INITIALIZE MOTORS
        // =================================

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ballPusher = hardwareMap.get(DcMotor.class, "ballPusher");

        servo0 = hardwareMap.get(Servo.class, "servo0");

        // =================================
        // MOTOR DIRECTIONS
        // =================================

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);

        // =================================
        // SHOOTER ENCODER MODE
        // =================================

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // =================================
            // MECANUM DRIVE CONTROLS
            // =================================

            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x * 1.1;
            turn = gamepad1.right_stick_x;

            // NORMAL SPEED
            pow1 = NORMAL_SPEED;
            pow2 = NORMAL_SPEED;

            // LEFT TRIGGER = PRECISION MODE
            if (gamepad1.left_trigger > 0.1) {
                pow1 = PRECISION_SPEED;
                pow2 = PRECISION_SPEED;
            }

            // RIGHT TRIGGER = REDUCED TURN SPEED
            if (gamepad1.right_trigger > 0.1) {
                pow2 = 0.25;
            }

            // =================================
            // SHOOTER CONTROL - 160 RPM
            // =================================

            if (gamepad1.a) {

                double targetTicksPerSecond =
                        SHOOTER_TARGET_RPM *
                                SHOOTER_TICKS_PER_REV / 60.0;

                shooter.setVelocity(targetTicksPerSecond);

            } else {
                shooter.setVelocity(0);
            }

            // ACTUAL SHOOTER RPM
            double actualRPM =
                    shooter.getVelocity() * 60.0 /
                            SHOOTER_TICKS_PER_REV;

            // =================================
            // BALL COUNTER
            // =================================

            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // ADD ONE BALL
            if (currentDpadUp && !previousDpadUp) {
                if (ballCount < 4) {
                    ballCount++;
                }
            }

            // REMOVE ONE BALL AFTER SHOOTING
            if (currentDpadDown && !previousDpadDown) {
                if (ballCount > 0) {
                    ballCount--;
                }
            }

            previousDpadUp = currentDpadUp;
            previousDpadDown = currentDpadDown;

            // =================================
            // BALL PUSHER / INTAKE
            // =================================

            String intakeStatus;

            // STOP INTAKE WHEN 4 BALLS ARE COUNTED
            if (ballCount >= 4) {

                ballPusher.setPower(0);
                intakeStatus = "FULL - STOPPED";

            } else if (gamepad1.x) {

                // X = INTAKE FORWARD ONLY
                ballPusher.setPower(1.0);
                intakeStatus = "FORWARD";

            } else {

                ballPusher.setPower(0);
                intakeStatus = "STOPPED";
            }

            // =================================
            // SERVO CONTROL
            // =================================

            boolean currentY = gamepad1.y;

            // PRESS Y = MOVE FORWARD
            if (currentY && !previousY) {
                servo0.setPosition(SERVO_REVERSE);
            }

            // RELEASE Y = RETURN TO START
            if (!currentY && previousY) {
                servo0.setPosition(SERVO_START);
            }

            previousY = currentY;

            // =================================
            // MECANUM MOTOR POWERS
            // =================================

            frontLeft.setPower(
                    (forward * pow1) + (turn * pow2) + (strafe * pow1)
            );

            backLeft.setPower(
                    (forward * pow1) + (turn * pow2) - (strafe * pow1)
            );

            frontRight.setPower(
                    (forward * pow1) - (turn * pow2) - (strafe * pow1)
            );

            backRight.setPower(
                    (forward * pow1) - (turn * pow2) + (strafe * pow1)
            );

            // =================================
            // DRIVER STATION TELEMETRY
            // =================================

            telemetry.addLine("=== HELIX HYDRAULICS 25879 ===");

            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData("Target RPM", SHOOTER_TARGET_RPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);

            telemetry.addLine("--- INTAKE ---");
            telemetry.addData("Intake Status", intakeStatus);
            telemetry.addData("Ball Count", "%d / 4", ballCount);

            telemetry.addLine("--- DRIVE ---");
            telemetry.addData(
                    "Drive Mode",
                    gamepad1.left_trigger > 0.1
                            ? "PRECISION (25%)"
                            : "NORMAL"
            );

            telemetry.addData("Servo Position", servo0.getPosition());

            telemetry.update();
        }

        // =================================
        // STOP ALL MOTORS
        // =================================

        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        shooter.setVelocity(0);
        ballPusher.setPower(0);
    }
}
**/
