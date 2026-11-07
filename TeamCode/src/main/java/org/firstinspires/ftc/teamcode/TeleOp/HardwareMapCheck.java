package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/** Zero-motion hardware map checks. Doesn't power any motor or servo. */
@TeleOp(name = "25879 TEST - Hardware Map", group = "Testing")
public final class HardwareMapCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("HARDWARE MAP CHECK - NEVER MOVES ANY DEVICE");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("backLeft", foundMotor("backLeft"));
            telemetry.addData("frontLeft", foundMotor("frontLeft"));
            telemetry.addData("backRight", foundMotor("backRight"));
            telemetry.addData("frontRight", foundMotor("frontRight"));
            telemetry.addData("shooter", foundMotor("shooter"));
            telemetry.addData("ballPusher", foundMotor("ballPusher"));
            telemetry.addData("servo0", hardwareMap.tryGet(Servo.class, "servo0") != null ? "FOUND" : "MISSING");
            telemetry.addData("intake (optional)", foundMotor("intake"));
            telemetry.addLine("No motor/servo commands are issued by this test.");
            telemetry.update();
            idle();
        }
    }
    private String foundMotor(String name) {
        return hardwareMap.tryGet(DcMotor.class, name) == null ? "MISSING" : "FOUND";
    }
}
