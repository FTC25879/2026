
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Android Shooter Control Hunter")
public class AndroidShooter extends LinearOpMode {

    private static final double MAX_TARGET_RPM = 6000;

    // Fine control: 10 RPM per click
    private static final double RPM_STEP = 10;

    // Gentle changes in commanded RPM
    private static final double RPM_RAMP_RATE = 200;

    // RPM stability threshold
    private static final double RPM_TOLERANCE = 10;

    // Report notable sudden RPM losses
    private static final double DROP_THRESHOLD = 20;

    @Override
    public void runOpMode() {

        DcMotorEx shooter = hardwareMap.get(
                DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);

        double ticksPerRev =
                shooter.getMotorType().getTicksPerRev();

        if (ticksPerRev <= 0) {
            telemetry.addLine("Invalid motor encoder");
            telemetry.update();
            return;
        }

        double ratedRPM =
                shooter.getMotorType().getMaxRPM();

        VoltageSensor battery = null;

        for (VoltageSensor sensor :
                hardwareMap.voltageSensor) {
            if (battery == null) {
                battery = sensor;
            }
        }

        double targetRPM = 0;
        double commandedRPM = 0;
        double actualRPM = 0;

        double peakRPM = 0;
        double peakAmps = 0;
        double previousRPM = 0;
        double largestDrop = 0;

        // Stores most recent significant RPM drop
        double lastDrop = 0;

        int activeGamepad = 0;

        boolean lastUp1 = false;
        boolean lastDown1 = false;
        boolean lastUp2 = false;
        boolean lastDown2 = false;

        boolean lastReset = false;

        // RPM control throughout operation
        shooter.setVelocity(0);

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("SHOOTER READY");
        telemetry.addLine("CLOSED LOOP RPM CONTROL");
        telemetry.addData(
                "Adjustment", "%.0f RPM", RPM_STEP);
        telemetry.addData(
                "Motor Rated RPM", "%.0f", ratedRPM);
        telemetry.update();

        waitForStart();
        timer.reset();

        try {
            while (opModeIsActive()) {

                double dt = Range.clip(
                        timer.seconds(), 0.001, 0.10);
                timer.reset();

                // GAMEPAD INPUT
                boolean up1 = gamepad1.dpad_up;
                boolean down1 = gamepad1.dpad_down;

                boolean up2 = gamepad2.dpad_up;
                boolean down2 = gamepad2.dpad_down;

                boolean pressUp1 = up1 && !lastUp1;
                boolean pressDown1 = down1 && !lastDown1;

                boolean pressUp2 = up2 && !lastUp2;
                boolean pressDown2 = down2 && !lastDown2;

                boolean stop =
                        gamepad1.a || gamepad2.a;

                boolean reset =
                        gamepad1.b || gamepad2.b;

                if (stop) {
                    targetRPM = 0;
                    commandedRPM = 0;
                }
                else if (pressUp1 || pressDown1) {
                    activeGamepad = 1;

                    if (pressUp1) targetRPM += RPM_STEP;
                    if (pressDown1) targetRPM -= RPM_STEP;
                }
                else if (pressUp2 || pressDown2) {
                    activeGamepad = 2;

                    if (pressUp2) targetRPM += RPM_STEP;
                    if (pressDown2) targetRPM -= RPM_STEP;
                }

                lastUp1 = up1;
                lastDown1 = down1;
                lastUp2 = up2;
                lastDown2 = down2;

                targetRPM = Range.clip(
                        targetRPM, 0, MAX_TARGET_RPM);

                // RPM RAMP - never directly jumps
                // to a newly requested target.
                double maxChange =
                        RPM_RAMP_RATE * dt;

                commandedRPM += Range.clip(
                        targetRPM - commandedRPM,
                        -maxChange,
                        maxChange);

                if (targetRPM == 0) {
                    commandedRPM = 0;
                }

                // Motor controller maintains RPM
                // using encoder velocity feedback.
                double ticksPerSecond =
                        commandedRPM * ticksPerRev / 60.0;

                shooter.setVelocity(ticksPerSecond);

                // ACTUAL RPM
                actualRPM =
                        Math.abs(shooter.getVelocity())
                                * 60.0 / ticksPerRev;

                // CURRENT IN AMPS
                double motorAmps =
                        shooter.getCurrent(CurrentUnit.AMPS);

                // BATTERY VOLTAGE
                double voltage =
                        battery != null
                                ? battery.getVoltage()
                                : Double.NaN;

                double rpmError =
                        targetRPM - actualRPM;

                // TRACK PEAK VALUES
                if (actualRPM > peakRPM) {
                    peakRPM = actualRPM;
                }

                if (motorAmps > peakAmps) {
                    peakAmps = motorAmps;
                }

                // DETECT SUDDEN RPM DROP
                double rpmDrop =
                        previousRPM - actualRPM;

                if (targetRPM > 0
                        && commandedRPM > 0
                        && rpmDrop >= DROP_THRESHOLD) {

                    lastDrop = rpmDrop;

                    if (rpmDrop > largestDrop) {
                        largestDrop = rpmDrop;
                    }
                }

                previousRPM = actualRPM;

                // RESET STATISTICS WITH B
                if (reset && !lastReset) {
                    peakRPM = actualRPM;
                    peakAmps = motorAmps;
                    largestDrop = 0;
                    lastDrop = 0;
                }

                lastReset = reset;

                // CURRENT PROTECTION STATUS
                boolean overCurrent =
                        shooter.isOverCurrent();

                // STATUS
                String status;

                if (targetRPM == 0) {
                    status = "STOPPED";
                }
                else if (overCurrent) {
                    status = "CURRENT ALERT";
                }
                else if (Math.abs(
                        targetRPM - commandedRPM) > 1) {
                    status = "RAMPING";
                }
                else if (Math.abs(rpmError)
                        <= RPM_TOLERANCE) {
                    status = "ON TARGET";
                }
                else if (rpmError > 0) {
                    status = "BELOW TARGET";
                }
                else {
                    status = "ABOVE TARGET";
                }

                // TELEMETRY
                telemetry.addLine(
                        "=== SHOOTER RPM CONTROL ===");

                telemetry.addData(
                        "TARGET RPM", "%.0f", targetRPM);

                telemetry.addData(
                        "COMMANDED RPM", "%.1f",
                        commandedRPM);

                telemetry.addData(
                        "ACTUAL RPM", "%.1f",
                        actualRPM);

                telemetry.addData(
                        "RPM ERROR", "%+.1f",
                        rpmError);

                telemetry.addData(
                        "PEAK RPM", "%.1f",
                        peakRPM);

                telemetry.addData(
                        "LAST RPM DROP", "%.1f",
                        lastDrop);

                telemetry.addData(
                        "LARGEST RPM DROP", "%.1f",
                        largestDrop);

                telemetry.addLine("");

                telemetry.addLine(
                        "=== ELECTRICAL ===");

                telemetry.addData(
                        "MOTOR CURRENT", "%.2f A",
                        motorAmps);

                telemetry.addData(
                        "PEAK CURRENT", "%.2f A",
                        peakAmps);

                telemetry.addData(
                        "OVERCURRENT",
                        overCurrent ? "YES" : "NO");

                telemetry.addData(
                        "BATTERY", "%.2f V",
                        voltage);

                telemetry.addLine("");

                telemetry.addLine(
                        "=== MOTOR STATUS ===");

                telemetry.addData("STATUS", status);

                telemetry.addData(
                        "RATED RPM", "%.0f",
                        ratedRPM);

                telemetry.addData(
                        "GAMEPAD", activeGamepad);

                telemetry.addLine("");

                telemetry.addLine(
                        "UP: +10 RPM");
                telemetry.addLine(
                        "DOWN: -10 RPM");
                telemetry.addLine(
                        "A: Stop");
                telemetry.addLine(
                        "B: Reset statistics");

                telemetry.update();

                sleep(20);
            }
        }
        finally {
            shooter.setVelocity(0);
        }
    }
}
