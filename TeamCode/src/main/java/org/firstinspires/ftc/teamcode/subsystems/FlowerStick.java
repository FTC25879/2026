package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Flower-removal stick controlled by the positional servo named "stick".
 * In the FTC Robot Configuration, map this servo to Expansion Hub Servo Port 5.
 *
 * Press gamepad2 Y once to extend, once again to retract.
 */
public class FlowerStick {

    // Initial, conservative TEST positions. Calibrate to your physical linkage.
    // Range is 0.0 to 1.0; larger is not necessarily "out" on your robot.
    public static final double RETRACTED_POSITION = 0.50;
    public static final double EXTENDED_POSITION = 0.60;

    private final Servo servo;
    private boolean extended = false;
    private boolean previousY = false;

    public FlowerStick(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "stick");
        retract();
    }

    /** Call once every TeleOp loop; use gamepad2. */
    public void update(Gamepad gamepad2) {
        boolean yPressed = gamepad2.y;
        if (yPressed && !previousY) {
            toggle();
        }
        previousY = yPressed;
    }

    public void toggle() {
        if (extended) {
            retract();
        } else {
            extend();
        }
    }

    public void extend() {
        servo.setPosition(EXTENDED_POSITION);
        extended = true;
    }

    public void retract() {
        servo.setPosition(RETRACTED_POSITION);
        extended = false;
    }

    /** Reports the last commanded position, not actual measured stick position. */
    public boolean isExtended() {
        return extended;
    }

    public boolean isRetracted() {
        return !extended;
    }
}
