
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Simple Servo Control", group = "Linear OpMode")
public class ServoTesting extends LinearOpMode {

    private double servoPosition = 0.5;

    @Override
    public void runOpMode() {

        Servo myServo = hardwareMap.get(Servo.class, "servo0");

        myServo.setPosition(servoPosition);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Move servo gradually using left joystick
            double joystick = gamepad2.left_stick_y;

            if (Math.abs(joystick) > 0.1) {
                servoPosition -= joystick * 0.01;
            }

            // Buttons for testing
            if (gamepad1.a) {
                servoPosition = 0.0;
            }

            if (gamepad1.b) {
                servoPosition = 0.5;
            }

            if (gamepad1.y) {
                servoPosition = 1.0;
            }

            // Keep position between 0 and 1
            servoPosition = Range.clip(servoPosition, 0.0, 1.0);

            myServo.setPosition(servoPosition);

            telemetry.addData("Gamepad Left Y", joystick);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addData("Gamepad A", gamepad2.a);
            telemetry.addData("Gamepad B", gamepad2.b);
            telemetry.addData("Gamepad Y", gamepad2.y);
            telemetry.update();

            sleep(20);
        }
    }
}
