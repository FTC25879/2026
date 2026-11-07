package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Verified legacy A/X/Y bindings alongside the tested robot-centric mecanum.
 * G1: sticks = drive, triggers = drive scaling, A = shooter, X = pusher, Y = servo.
 * G2: RT = intake forward, LT = intake reverse (optional 'intake' hardware name).
 * Does not run automatic shot sequencing or RPM closed-loop logic.
 */
//@TeleOp(name = "25879 Competition TeleOp", group = "Competition")
public final class CompetitionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Manual mode maps only hardware confirmed in the working OnMecanumDrive.
        Robot robot = new Robot(hardwareMap, true);
        try {
            telemetry.addLine("25879 - ORIGINAL CONTROLS + MECANUM");
            telemetry.addLine("G1 sticks = drive; LT/RT = scale drive/turn");
            telemetry.addLine("G1 hold A = shooter; hold X = ballPusher");
            telemetry.addLine("G1 hold Y = servo0 0.50; release = 0.189");
            telemetry.addLine("G2 RT/LT = intake forward/reverse (optional)");
            telemetry.addData("Intake found", robot.manualMechanisms.hasIntake());
            telemetry.addData("Intake hardware name", robot.manualMechanisms.intakeName());
            telemetry.addLine("Full-power shooter/pusher: test with mechanisms clear.");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive()) {
                robot.manualDrive.drive(gamepad1);
                robot.manualMechanisms.update(gamepad1, gamepad2);

                telemetry.addData("G1 Shooter A", gamepad1.a ? "ON 1.0" : "OFF");
                telemetry.addData("G1 Pusher X", gamepad1.x ? "ON 1.0" : "OFF");
                telemetry.addData("G1 Servo Y", gamepad1.y ? "0.50 command" : "0.189 after release");
                telemetry.addData("Servo command", "%.3f", robot.manualMechanisms.servoCommand());
                telemetry.addData("Intake found", robot.manualMechanisms.hasIntake());
                telemetry.addData("G2 intake command", "%.2f", robot.manualMechanisms.intakePower());
                telemetry.addData("Battery (V)", "%.2f", robot.voltage());
                telemetry.addLine("G1 triggers scale drive; G2 triggers control intake.");
                telemetry.update();
                idle();
            }
        } finally {
            robot.stopAll();
        }
    }
}
