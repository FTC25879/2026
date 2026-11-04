package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.control.HeadingAim;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

/**
 * Competition driver mode. One gamepad mode can be selected BEFORE Start with G1 X.
 * G1: drive, left bumper locks current heading, right bumper precision drive.
 * G2 (or G1 single-mode): right/left triggers intake/reverse, Y toggles stick,
 * A request one shot, D-pad U/D target RPM +/-10, D-pad R start, L stop, X fault reset (only stopped).
 */
@TeleOp(name = "25879 Competition TeleOp", group = "Competition")
public final class CompetitionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        HeadingAim headingAim = new HeadingAim();
        boolean singleController = false;
        boolean lastSetupX = false;
        boolean lastAim = false;
        boolean lastUp = false, lastDown = false, lastA = false;
        boolean lastRight = false, lastLeft = false, lastX = false;

        try {
            while (!isStarted() && !isStopRequested()) {
                boolean x = gamepad1.x;
                if (x && !lastSetupX) singleController = !singleController;
                lastSetupX = x;
                telemetry.addLine("25879 - COMPETITION TELEOP");
                telemetry.addData("Controllers", singleController ? "ONE (G1 drives + mechanisms)" : "TWO (G1 drive, G2 mechanisms)");
                telemetry.addLine("G1 X toggles one/two controller mode before START");
                telemetry.addLine("Verify servo positions, shooter ticks/rev and current limits before shooting!");
                telemetry.update();
                sleep(50);
            }
            if (isStopRequested()) return;
            headingAim.capture(robot.headingRadians());

            while (opModeIsActive()) {
                Gamepad mechanism = singleController ? gamepad1 : gamepad2;

                // Drive. Left bumper holds the heading captured on its first press.
                boolean aim = gamepad1.left_bumper;
                if (aim && !lastAim) headingAim.capture(robot.headingRadians());
                lastAim = aim;
                double forward = -gamepad1.left_stick_y;
                double strafe = -gamepad1.left_stick_x;
                double turn = aim ? headingAim.getTurn(robot.headingRadians()) : -gamepad1.right_stick_x;
                double speed = gamepad1.right_bumper ? 0.28 : 0.78;
                robot.drive.drive(new DrivePowers(forward * speed, strafe * speed, turn * speed), true);

                // Intake only runs while a trigger is being held.
                if (mechanism.right_trigger > 0.2 && mechanism.left_trigger <= 0.2) robot.intake.start();
                else if (mechanism.left_trigger > 0.2 && mechanism.right_trigger <= 0.2) robot.intake.reverse();
                else robot.intake.stop();

                boolean up = mechanism.dpad_up, down = mechanism.dpad_down;
                boolean right = mechanism.dpad_right, left = mechanism.dpad_left;
                boolean a = mechanism.a, x = mechanism.x;
                if (up && !lastUp) robot.shooter.adjustRPM(RobotConfig.SHOOTER_STEP_RPM);
                if (down && !lastDown) robot.shooter.adjustRPM(-RobotConfig.SHOOTER_STEP_RPM);
                if (right && !lastRight) robot.shooter.start();
                if (left && !lastLeft) { robot.shots.cancel(); robot.shooter.stop(); }
                if (a && !lastA) robot.shots.requestOne();
                // X only clears a latched fault when motor is stopped. Start with D-pad right.
                if (x && !lastX && !robot.shooter.isEnabled()) robot.shooter.clearFault();
                lastUp = up; lastDown = down; lastRight = right; lastLeft = left;
                lastA = a; lastX = x;

                robot.stick.update(mechanism); // Y toggles, includes key-edge detection
                robot.update();
                TelemetryManager.addRobot(telemetry, robot);
                telemetry.addData("Control mode", singleController ? "ONE" : "TWO");
                telemetry.addLine("R/L triggers intake | D-pad U/D RPM | R start | L stop");
                telemetry.addLine("A feed one | Y stick | X clear stopped fault");
                telemetry.update();
                idle();
            }
        } finally {
            robot.stopAll();
        }
    }
}
