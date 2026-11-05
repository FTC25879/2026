package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Independent diagnostic for the positional servo named "stick".
 * Does not construct Robot, access IMU, or run any motors.
 * Configure "stick" as Servo at Expansion Hub servo port 5.
 * With linkage disconnected, test a limited travel range first.
 */
@TeleOp(name = "25879 Stick ONLY Test", group = "Diagnostics")
public class StickOnlyTest extends LinearOpMode {
    private static final double CENTER = 0.50;
    private static final double TEST_EXTENDED = 0.60;
    private static final double STEP = 0.02;
    private static final double SAFE_MIN = 0.35;
    private static final double SAFE_MAX = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo stick;
        try {
            stick = hardwareMap.get(Servo.class, "stick");
        } catch (RuntimeException e) {
            telemetry.addLine("ERROR: Can't find positional Servo named 'stick'");
            telemetry.addLine("Check Expansion Hub / Servo Port 5 configuration.");
            telemetry.addData("Details", e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addLine("Found Servo 'stick'. Linkage should be free.");
        telemetry.addLine("G2 Y (triangle): toggle 0.50 / 0.60");
        telemetry.addLine("G1 Y works too, only as a diagnostic fallback.");
        telemetry.addLine("G2 D-pad UP/DOWN: change by 0.02");
        telemetry.addLine("Press PLAY to begin. Nothing moves during INIT.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        double requested = CENTER;
        stick.setPosition(requested);
        boolean lastG1Y = false, lastG2Y = false;
        boolean lastUp = false, lastDown = false;
        String action = "Started at center";

        while (opModeIsActive()) {
            boolean g1y = gamepad1.y;
            boolean g2y = gamepad2.y;
            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;

            if ((g2y && !lastG2Y) || (g1y && !lastG1Y)) {
                requested = requested >= 0.55 ? CENTER : TEST_EXTENDED;
                action = g2y ? "G2 Y toggle" : "G1 Y toggle (fallback)";
                stick.setPosition(requested);
            } else if (up && !lastUp) {
                requested = Range.clip(requested + STEP, SAFE_MIN, SAFE_MAX);
                action = "G2 D-pad UP (+0.02)";
                stick.setPosition(requested);
            } else if (down && !lastDown) {
                requested = Range.clip(requested - STEP, SAFE_MIN, SAFE_MAX);
                action = "G2 D-pad DOWN (-0.02)";
                stick.setPosition(requested);
            }

            lastG1Y = g1y;
            lastG2Y = g2y;
            lastUp = up;
            lastDown = down;

            telemetry.addLine("25879 STICK DIAGNOSTIC");
            telemetry.addData("G2 Triangle/Y received", g2y);
            telemetry.addData("G1 Triangle/Y received", g1y);
            telemetry.addData("Last action", action);
            telemetry.addData("Commanded position", "%.2f", requested);
            telemetry.addData("Servo API position", "%.2f", stick.getPosition());
            telemetry.addLine("Servo API position is NOT physical feedback.");
            telemetry.addLine("If values change but servo doesn't, check cable/power/jam.");
            telemetry.update();
            idle();
        }
    }
}
