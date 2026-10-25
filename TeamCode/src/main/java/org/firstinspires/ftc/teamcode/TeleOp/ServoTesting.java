package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Simple Servo Control", group = "Linear OpMode")
public class ServoTesting extends LinearOpMode {

    // Declare the servo
    private Servo myServo;

    @Override
    public void runOpMode() {
        // Map the servo to port/channel 0 as configured in the robot configuration
        myServo = hardwareMap.get(Servo.class, "servo0");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Control servo position using Gamepad 1 buttons
            if (gamepad1.a) {
                myServo.setPosition(1.0); // Move to position 1.0 (Max)
            } else if (gamepad1.b) {
                myServo.setPosition(0.0); // Move to position 0.0 (Min)
            } else if (gamepad1.x) {
                myServo.setPosition(0.5); // Move to position 0.5 (Center)
            }

            // Send position data back to the driver station
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}
