package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * FTC 25879: Manual, slow calibration of the REV Smart Robot Servo V2.
 * Configured as a standard positional Servo named "stick" on Expansion Hub port 5.
 *
 * This test allows the full normalized servo-command range. It DOES NOT sense
 * mechanical end stops: stop moving before the linkage binds or stalls.
 */
@TeleOp(name = "25879 Stick Calibration")
public class ServoHardwareTest extends LinearOpMode {

    private static final double COMMAND_MIN = 0.0;
    private static final double COMMAND_MAX = 1.0;
    private static final double START_POSITION = 0.50;

    // Speed is in normalized servo position per second, NOT servo motor RPM.
    private static final double START_SPEED = 0.10;
    private static final double SPEED_STEP = 0.025;
    private static final double MIN_SPEED = 0.01;
    private static final double MAX_SPEED = 0.50;
    private static final double JOYSTICK_DEADZONE = 0.08;

    @Override
    public void runOpMode() {
        Servo stick = hardwareMap.get(Servo.class, "stick");
        Servo.Direction direction = Servo.Direction.FORWARD;
        stick.setDirection(direction);

        double commandPosition = START_POSITION;
        double speed = START_SPEED;
        double savedRetracted = Double.NaN;
        double savedExtended = Double.NaN;
        boolean previousX = false;
        boolean previousUp = false;
        boolean previousDown = false;
        boolean previousLB = false;
        boolean previousRB = false;

        telemetry.addLine("25879 Stick Calibration");
        telemetry.addLine("Gamepad 2 LEFT STICK up/down: move while held");
        telemetry.addLine("D-pad UP/DOWN: faster/slower");
        telemetry.addLine("X: toggle FORWARD / REVERSE");
        telemetry.addLine("Left bumper: save RETRACTED endpoint");
        telemetry.addLine("Right bumper: save EXTENDED endpoint");
        telemetry.addLine("STOP before reaching a mechanical hard stop!");
        telemetry.addLine("Servo will move to 0.50 at START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // This sets a commanded position; a standard servo has no position feedback.
        stick.setPosition(commandPosition);
        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        while (opModeIsActive()) {
            double dt = Range.clip(cycleTimer.seconds(), 0.0, 0.05);
            cycleTimer.reset();

            boolean x = gamepad2.x;
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;
            boolean lb = gamepad2.left_bumper;
            boolean rb = gamepad2.right_bumper;

            // Reverse the command mapping WITHOUT causing a sudden servo jump.
            // The FTC Servo API maps reverse to 1.0 - command position.
            if (x && !previousX) {
                commandPosition = 1.0 - commandPosition;
                direction = (direction == Servo.Direction.FORWARD)
                        ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
                stick.setDirection(direction);
                stick.setPosition(commandPosition);
            }

            if (up && !previousUp) speed += SPEED_STEP;
            if (down && !previousDown) speed -= SPEED_STEP;
            speed = Range.clip(speed, MIN_SPEED, MAX_SPEED);

            // Up increases logical position; down decreases it.
            // X toggles which physical direction corresponds to increasing position.
            double joystick = -gamepad2.left_stick_y;
            if (Math.abs(joystick) < JOYSTICK_DEADZONE) joystick = 0;

            commandPosition += joystick * speed * dt;
            commandPosition = Range.clip(commandPosition, COMMAND_MIN, COMMAND_MAX);
            stick.setPosition(commandPosition);

            // This number stays in the same reference frame when X reverses mapping.
            // It is a commanded position, NOT an actual angle measurement.
            double forwardEquivalent = (direction == Servo.Direction.FORWARD)
                    ? commandPosition : (1.0 - commandPosition);

            if (lb && !previousLB) savedRetracted = forwardEquivalent;
            if (rb && !previousRB) savedExtended = forwardEquivalent;

            telemetry.addData("Gamepad 2 X (reverse)", x);
            telemetry.addData("Direction", direction);
            telemetry.addData("Joystick", "%.2f", joystick);
            telemetry.addData("Speed (position/sec)", "%.3f", speed);
            telemetry.addData("Logical command", "%.3f", commandPosition);
            telemetry.addData("Forward-equivalent command", "%.3f", forwardEquivalent);
            telemetry.addData("Saved RETRACTED", Double.isNaN(savedRetracted) ? "not saved" : String.format("%.3f", savedRetracted));
            telemetry.addData("Saved EXTENDED", Double.isNaN(savedExtended) ? "not saved" : String.format("%.3f", savedExtended));
            telemetry.addLine("Stop before the stick touches or binds!");
            telemetry.update();

            previousX = x;
            previousUp = up;
            previousDown = down;
            previousLB = lb;
            previousRB = rb;
            idle();
        }
    }
}
