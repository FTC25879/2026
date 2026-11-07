
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name = "Simple Stick Test")
public class ServoHardwareTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo stick = hardwareMap.get(Servo.class, "stick");

        // Start from the servo's last commanded position.
        double position = stick.getPosition();

        if (Double.isNaN(position)) {
            position = 0.50;
        }

        // Small movement per loop
        double speed = 0.003;

        boolean lastUp = false;
        boolean lastDown = false;

        telemetry.addLine("Stick ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // UP increases movement speed
            if (gamepad1.dpad_up && !lastUp) {
                speed += 0.001;
            }

            // DOWN decreases movement speed
            if (gamepad1.dpad_down && !lastDown) {
                speed -= 0.001;
            }

            speed = Range.clip(speed, 0.001, 0.020);

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // Left joystick controls direction
            double joystick = gamepad1.left_stick_x;

            if (Math.abs(joystick) > 0.15) {

                // Move a small amount each loop
                position += joystick * speed;

                // Keep inside servo command range
                position = Range.clip(position, 0.0, 1.0);

                stick.setPosition(position);
            }

            telemetry.addData("Position", "%.3f", position);
            telemetry.addData("Speed", "%.3f", speed);
            telemetry.addData("Joystick", "%.2f", joystick);
            telemetry.update();

            sleep(25);
        }
    }
}
