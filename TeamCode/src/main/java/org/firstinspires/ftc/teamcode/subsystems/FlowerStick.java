
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flower-removal stick servo.
 *
 * Hardware configuration:
 * Name: stick
 * Expansion Hub: Servo Port 5
 *
 * GAMEPAD 2:
 *
 * Y = Extend / Retract
 * X = Reverse direction
 *
 * D-pad UP    = Increase MAX position
 * D-pad DOWN  = Decrease MAX position
 * D-pad RIGHT = Increase MIN position
 * D-pad LEFT  = Decrease MIN position
 *
 * Left bumper  = Fine adjustment (0.001)
 * Right bumper = Coarse adjustment (0.05)
 * Normal adjustment = 0.01
 *
 * All values are adjustable during TeleOp.
 */
public final class FlowerStick {

    // Initial calibration values
    public static final double DEFAULT_MIN = 0.50;
    public static final double DEFAULT_MAX = 0.60;
    public static final boolean DEFAULT_REVERSED = false;

    // Adjustment settings
    private static final double NORMAL_STEP = 0.01;
    private static final double FINE_STEP = 0.001;
    private static final double COARSE_STEP = 0.05;

    // Minimum distance between endpoints
    private static final double MIN_GAP = 0.01;

    // D-pad repeat interval when held
    private static final double REPEAT_SECONDS = 0.18;

    private final Servo servo;
    private final ElapsedTime repeatTimer = new ElapsedTime();

    // Adjustable calibration
    private double minPosition = DEFAULT_MIN;
    private double maxPosition = DEFAULT_MAX;
    private boolean reversed = DEFAULT_REVERSED;

    // Current state
    private boolean extended = false;

    // Button edge detection
    private boolean previousY = false;
    private boolean previousX = false;

    private int previousDpadCommand = 0;

    public FlowerStick(HardwareMap hardwareMap) {

        servo = hardwareMap.get(Servo.class, "stick");

        // Start retracted
        retract();
    }

    /**
     * Call every TeleOp loop using gamepad2.
     */
    public void update(Gamepad gamepad2) {

        // ----------------------------
        // Y: Extend / retract
        // ----------------------------
        boolean yPressed = gamepad2.y;

        if (yPressed && !previousY) {
            toggle();
        }

        previousY = yPressed;

        // ----------------------------
        // X: Reverse direction
        // ----------------------------
        boolean xPressed = gamepad2.x;

        if (xPressed && !previousX) {
            reversed = !reversed;
            applyPosition();
        }

        previousX = xPressed;

        // ----------------------------
        // Select adjustment speed
        // ----------------------------
        double step = NORMAL_STEP;

        if (gamepad2.left_bumper) {
            step = FINE_STEP;
        }

        if (gamepad2.right_bumper) {
            step = COARSE_STEP;
        }

        // ----------------------------
        // Read D-pad command
        // ----------------------------
        int command = 0;

        if (gamepad2.dpad_up) {
            command = 1;
        } else if (gamepad2.dpad_down) {
            command = 2;
        } else if (gamepad2.dpad_right) {
            command = 3;
        } else if (gamepad2.dpad_left) {
            command = 4;
        }

        // First press responds immediately.
        // Holding repeats at a controlled rate.
        if (command != 0 &&
                (command != previousDpadCommand ||
                        repeatTimer.seconds() >= REPEAT_SECONDS)) {

            switch (command) {

                case 1:
                    // Increase maximum
                    maxPosition = Math.min(
                            1.0,
                            maxPosition + step
                    );
                    break;

                case 2:
                    // Decrease maximum
                    maxPosition = Math.max(
                            minPosition + MIN_GAP,
                            maxPosition - step
                    );
                    break;

                case 3:
                    // Increase minimum
                    minPosition = Math.min(
                            maxPosition - MIN_GAP,
                            minPosition + step
                    );
                    break;

                case 4:
                    // Decrease minimum
                    minPosition = Math.max(
                            0.0,
                            minPosition - step
                    );
                    break;
            }

            // Move to updated endpoint immediately
            applyPosition();

            repeatTimer.reset();
        }

        previousDpadCommand = command;
    }

    /**
     * Move servo based on state and direction.
     */
    private void applyPosition() {

        double target;

        if (!reversed) {
            target = extended
                    ? maxPosition
                    : minPosition;
        } else {
            target = extended
                    ? minPosition
                    : maxPosition;
        }

        servo.setPosition(target);
    }

    /**
     * Toggle between extended and retracted.
     */
    public void toggle() {

        if (extended) {
            retract();
        } else {
            extend();
        }
    }

    /**
     * Extend the stick.
     */
    public void extend() {
        extended = true;
        applyPosition();
    }

    /**
     * Retract the stick.
     */
    public void retract() {
        extended = false;
        applyPosition();
    }

    /**
     * Reverse which endpoint means extended.
     */
    public void setReversed(boolean reverse) {
        reversed = reverse;
        applyPosition();
    }

    public boolean isReversed() {
        return reversed;
    }

    public boolean isExtended() {
        return extended;
    }

    public boolean isRetracted() {
        return !extended;
    }

    public double getMinPosition() {
        return minPosition;
    }

    public double getMaxPosition() {
        return maxPosition;
    }

    public double getExtendedPosition() {
        return reversed ? minPosition : maxPosition;
    }

    public double getRetractedPosition() {
        return reversed ? maxPosition : minPosition;
    }

    /**
     * Last commanded servo position.
     * Not measured physical feedback.
     */
    public double getPosition() {
        return servo.getPosition();
    }

    /**
     * Add calibration information to telemetry.
     */
    public void addTelemetry(Telemetry telemetry) {

        telemetry.addData(
                "Stick State",
                extended ? "EXTENDED" : "RETRACTED"
        );

        telemetry.addData(
                "Stick Direction",
                reversed ? "REVERSED" : "NORMAL"
        );

        telemetry.addData(
                "Stick MIN",
                "%.3f", minPosition
        );

        telemetry.addData(
                "Stick MAX",
                "%.3f", maxPosition
        );

        telemetry.addData(
                "Retracted Target",
                "%.3f", getRetractedPosition()
        );

        telemetry.addData(
                "Extended Target",
                "%.3f", getExtendedPosition()
        );

        telemetry.addData(
                "Current Command",
                "%.3f", servo.getPosition()
        );
    }
}
