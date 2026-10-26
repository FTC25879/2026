
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter Control")
public class Shooter extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Expansion Hub motor port 3 (fourth connector)
        DcMotorEx shooter = hardwareMap.get(
                DcMotorEx.class, "shooter"
        );

        // Automatically obtain configured motor specifications
        double ticksPerRev =
                shooter.getMotorType().getTicksPerRev();

        double maxRPM =
                shooter.getMotorType().getMaxRPM();

        if (ticksPerRev <= 0 || maxRPM <= 0) {
            telemetry.addLine("ERROR: Select correct motor type");
            telemetry.update();
            return;
        }

        // Adapt increments to motor speed
        double rpmStep;

        if (maxRPM <= 200) {
            rpmStep = 10;       // HD Hex 40:1
        } else if (maxRPM <= 400) {
            rpmStep = 25;       // HD Hex 20:1
        } else {
            rpmStep = 250;      // High-speed motor
        }

        double targetRPM = 0;
        int activeGamepad = 0;

        boolean lastUp1 = false;
        boolean lastDown1 = false;
        boolean lastUp2 = false;
        boolean lastDown2 = false;

        shooter.setPower(0);
        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );
        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        telemetry.addLine("SHOOTER READY");
        telemetry.addData("Max RPM", "%.0f", maxRPM);
        telemetry.addData("Step", "%.0f RPM", rpmStep);
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Automatically accept either gamepad
                boolean up1 = gamepad1.dpad_up;
                boolean down1 = gamepad1.dpad_down;
                boolean up2 = gamepad2.dpad_up;
                boolean down2 = gamepad2.dpad_down;

                boolean pressUp1 = up1 && !lastUp1;
                boolean pressDown1 = down1 && !lastDown1;
                boolean pressUp2 = up2 && !lastUp2;
                boolean pressDown2 = down2 && !lastDown2;

                // Stop from either gamepad
                if (gamepad1.a || gamepad2.a) {
                    targetRPM = 0;
                    activeGamepad = gamepad1.a ? 1 : 2;
                }
                else if (pressUp1 || pressDown1) {
                    activeGamepad = 1;

                    if (pressUp1) targetRPM += rpmStep;
                    if (pressDown1) targetRPM -= rpmStep;
                }
                else if (pressUp2 || pressDown2) {
                    activeGamepad = 2;

                    if (pressUp2) targetRPM += rpmStep;
                    if (pressDown2) targetRPM -= rpmStep;
                }

                lastUp1 = up1;
                lastDown1 = down1;
                lastUp2 = up2;
                lastDown2 = down2;

                targetRPM = Range.clip(
                        targetRPM, 0, maxRPM
                );

                // Closed-loop RPM control
                double targetVelocity =
                        targetRPM * ticksPerRev / 60.0;

                if (targetRPM == 0) {
                    shooter.setPower(0);
                } else {
                    shooter.setVelocity(targetVelocity);
                }

                // Actual RPM measured by encoder
                double actualRPM =
                        Math.abs(shooter.getVelocity())
                                * 60.0 / ticksPerRev;

                telemetry.addLine("=== SHOOTER ===");

                telemetry.addData(
                        "TARGET RPM", "%.0f", targetRPM
                );
                telemetry.addData(
                        "ACTUAL RPM", "%.0f", actualRPM
                );
                telemetry.addData(
                        "RPM ERROR", "%.0f",
                        targetRPM - actualRPM
                );
                telemetry.addData(
                        "MAX RPM", "%.0f", maxRPM
                );
                telemetry.addData(
                        "RPM STEP", "%.0f", rpmStep
                );
                telemetry.addData(
                        "GAMEPAD", activeGamepad
                );

                telemetry.addLine("UP: Faster");
                telemetry.addLine("DOWN: Slower");
                telemetry.addLine("A: Stop");

                telemetry.update();
                sleep(20);
            }
        }
        finally {
            shooter.setPower(0);
        }
    }
}
