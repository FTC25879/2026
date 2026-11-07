package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * HELIX #25879 - autonomous WITHOUT shooting.
 * Drives forward 12 inches at 25% power using DRIVE MOTOR ENCODERS.
 * IMPORTANT: set DRIVE_TICKS_PER_INCH for YOUR motor gearing and wheels.
 */
@Autonomous(name = "Auto No Shooter", group = "Helix")
public class AutoNoShooter extends LinearOpMode {

    private static final double FORWARD_INCHES = 13.0;
    private static final double DRIVE_POWER = 0.25;

    // EXAMPLE only: REV HD Hex 20:1 (560 output ticks/rev) + 96 mm wheels.
    // For 40:1, this is roughly DOUBLE. Calibrate using a tape measure.
    private static final double DRIVE_TICKS_PER_INCH = 560.0 / (Math.PI * (96.0 / 25.4));
    private static final int POSITION_TOLERANCE_TICKS = 25;
    private static final double AUTO_LIMIT_SECONDS = 29.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor ballPusher;
    private Servo gate;
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ballPusher = hardwareMap.get(DcMotor.class, "ballPusher");
        gate = hardwareMap.get(Servo.class, "gate");

        // EXACT same physical directions as NewMecanumDrive TeleOp.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        shooter.setPower(0);
        ballPusher.setPower(0);
        gate.setPosition(0.189);

        telemetry.addLine("HELIX AUTO - NO SHOOTER");
        telemetry.addData("Forward", "%.1f inches", FORWARD_INCHES);
        telemetry.addData("Drive speed", "%.0f%%", DRIVE_POWER * 100.0);
        telemetry.addData("Encoder ticks/in", "%.2f (CALIBRATE!)", DRIVE_TICKS_PER_INCH);
        telemetry.addLine("Requires ALL 4 drive motor encoders connected.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        matchTimer.reset();
        try {
            boolean arrived = moveForward(FORWARD_INCHES, DRIVE_POWER);
            telemetry.addData("Result", arrived ? "DONE - STOPPED" : "ABORTED - CHECK DRIVE ENCODERS");
            telemetry.update();
        } finally {
            stopAllMotors();
        }
    }

    private boolean moveForward(double inches, double speed) {
        int delta = -(int) Math.round(inches * DRIVE_TICKS_PER_INCH);
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};
        int[] targets = new int[motors.length];
        int[] starts = new int[motors.length];

        for (int i = 0; i < motors.length; i++) {
            starts[i] = motors[i].getCurrentPosition();
            targets[i] = starts[i] + delta;
            motors[i].setTargetPosition(targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(Math.abs(speed));
        }

        ElapsedTime moveTimer = new ElapsedTime();
        double timeout = Math.max(3.0, inches / (10.0 * Math.abs(speed)) + 2.0);
        boolean encoderMoved = false;
        boolean arrived = false;
        while (opModeIsActive() && matchTimer.seconds() < AUTO_LIMIT_SECONDS
                && moveTimer.seconds() < timeout) {
            int maxError = 0;
            int minTravel = Integer.MAX_VALUE;
            for (int i = 0; i < motors.length; i++) {
                int position = motors[i].getCurrentPosition();
                maxError = Math.max(maxError, Math.abs(targets[i] - position));
                minTravel = Math.min(minTravel, Math.abs(position - starts[i]));
            }
            if (minTravel > 15) encoderMoved = true;
            if (maxError <= POSITION_TOLERANCE_TICKS) {
                arrived = true;
                break;
            }
            if (moveTimer.seconds() > 0.7 && !encoderMoved) break;
            telemetry.addData("AUTO", "Forward %.1f inches @ 25%%", inches);
            telemetry.addData("Encoder remaining", "%d ticks", maxError);
            telemetry.addData("Seconds", "%.1f", moveTimer.seconds());
            telemetry.update();
            sleep(20);
        }
        stopDrive();
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return arrived;
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void stopAllMotors() {
        stopDrive();
        shooter.setPower(0);
        ballPusher.setPower(0);
        gate.setPosition(0.189);
    }
}
