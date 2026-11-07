package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.WorkingMecanumDrive;

/** Drive-only verification: requires JUST the four configured mecanum motors. */
@TeleOp(name = "25879 TEST - Drive Only", group = "Testing")
public final class MecanumDriveOnlyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WorkingMecanumDrive drive = new WorkingMecanumDrive(hardwareMap);
        try {
            telemetry.addLine("DRIVE ONLY: Raise wheels before testing.");
            telemetry.addLine("G1 left stick forward/strafe, right stick turn.");
            telemetry.addLine("G1 left/right triggers scale movement.");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                drive.drive(gamepad1);
                telemetry.addData("Forward", "%.2f", -gamepad1.left_stick_y);
                telemetry.addData("Strafe", "%.2f", gamepad1.left_stick_x * 1.1);
                telemetry.addData("Turn", "%.2f", gamepad1.right_stick_x);
                telemetry.update();
                idle();
            }
        } finally {
            drive.stop();
        }
    }
}
