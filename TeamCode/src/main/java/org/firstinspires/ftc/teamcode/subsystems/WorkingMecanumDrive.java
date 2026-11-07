package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Robot-centric mecanum drive copied from the known-working OnMecanumDrive.
 * The wheel names, motor directions, joystick signs, 1.1 strafe factor,
 * trigger scaling, and wheel-power formulas are deliberately unchanged.
 * Does NOT initialize Pedro or require an IMU.
 */
public final class WorkingMecanumDrive {
    private final DcMotor backLeft;
    private final DcMotor frontLeft;
    private final DcMotor backRight;
    private final DcMotor frontRight;

    public WorkingMecanumDrive(HardwareMap hardwareMap) {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
    }

    /** Called once every TeleOp loop with the DRIVER gamepad (gamepad1). */
    public void drive(Gamepad gamepad) {
        // Exact inputs from OnMecanumDrive.
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x * 1.1;
        double turn = gamepad.right_stick_x;

        double pow1 = 1.0;
        double pow2 = 1.0;

        if (gamepad.left_trigger > 0.1) {
            pow1 = 0.75 - (gamepad.left_trigger / 2.0);
        }
        if (gamepad.right_trigger > 0.1) {
            pow2 = 0.75 - (gamepad.right_trigger / 2.0);
        }

        // Exact power mixing from OnMecanumDrive; no new normalization.
        frontLeft.setPower((forward * pow1) + (turn * pow2) + (strafe * pow1));
        backLeft.setPower((forward * pow1) + (turn * pow2) - (strafe * pow1));
        frontRight.setPower((forward * pow1) - (turn * pow2) - (strafe * pow1));
        backRight.setPower((forward * pow1) - (turn * pow2) + (strafe * pow1));
    }

    public void stop() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }
}
