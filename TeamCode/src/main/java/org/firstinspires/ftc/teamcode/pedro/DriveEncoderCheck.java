package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Slow driver-controlled test for calibrating encoder ticks/inch. */
@TeleOp(name = "Pedro 3 - Drive Encoder Check", group = "Diagnostics")
public class DriveEncoderCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontLeftName.get());
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.frontRightName.get());
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backLeftName.get());
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, Constants.drivetrainConfig.backRightName.get());
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(Constants.HUB_ORIENTATION));

        telemetry.addLine("Place robot at a measured starting line.");
        telemetry.addLine("A = reset tick baseline. Drive slowly 24 inches.");
        telemetry.update();
        waitForStart();
        int startFL = Constants.ENCODER_FL_SIGN * fl.getCurrentPosition();
        int startFR = Constants.ENCODER_FR_SIGN * fr.getCurrentPosition();
        int startBL = Constants.ENCODER_BL_SIGN * bl.getCurrentPosition();
        int startBR = Constants.ENCODER_BR_SIGN * br.getCurrentPosition();
        boolean oldA = false;
        try {
            while (opModeIsActive()) {
                boolean a = gamepad1.a;
                if (a && !oldA) {
                    startFL = Constants.ENCODER_FL_SIGN * fl.getCurrentPosition();
                    startFR = Constants.ENCODER_FR_SIGN * fr.getCurrentPosition();
                    startBL = Constants.ENCODER_BL_SIGN * bl.getCurrentPosition();
                    startBR = Constants.ENCODER_BR_SIGN * br.getCurrentPosition();
                }
                oldA = a;
                int dFL = Constants.ENCODER_FL_SIGN * fl.getCurrentPosition() - startFL;
                int dFR = Constants.ENCODER_FR_SIGN * fr.getCurrentPosition() - startFR;
                int dBL = Constants.ENCODER_BL_SIGN * bl.getCurrentPosition() - startBL;
                int dBR = Constants.ENCODER_BR_SIGN * br.getCurrentPosition() - startBR;
                double forwardTicks = (dFL + dFR + dBL + dBR) / 4.0;
                double strafeTicks = (-dFL + dFR + dBL - dBR) / 4.0;
                telemetry.addData("FL FR BL BR ticks", "%d %d %d %d", dFL, dFR, dBL, dBR);
                telemetry.addData("FORWARD ticks", "%.1f", forwardTicks);
                telemetry.addData("LEFT STRAFE ticks", "%.1f", strafeTicks);
                telemetry.addData("IMU yaw degrees", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addLine("For 24 in forward: FORWARD ticks / 24");
                telemetry.addLine("For 24 in left: LEFT STRAFE ticks / 24");
                telemetry.update();
                drive.drive(new DrivePowers(
                        -gamepad1.left_stick_y * 0.20,
                        -gamepad1.left_stick_x * 0.20,
                        -gamepad1.right_stick_x * 0.20), true);
            }
        } finally {
            drive.stop();
        }
    }
}
