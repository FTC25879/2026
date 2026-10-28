
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Android Shooter Control Hunter")
public class AndroidShooter extends LinearOpMode {

    private static final double MAX_TARGET_RPM = 6000;
    private static final double RPM_STEP = 250;

    @Override
    public void runOpMode() {

        DcMotorEx shooter = hardwareMap.get(
                DcMotorEx.class, "shooter"
        );

        // Reverse motor direction
        shooter.setDirection(DcMotor.Direction.REVERSE);

        double ticksPerRev =
                shooter.getMotorType().getTicksPerRev();

        if (ticksPerRev <= 0) {
            telemetry.addLine("ERROR: Invalid motor encoder");
            telemetry.update();
            return;
        }

        double ratedRPM =
                shooter.getMotorType().getMaxRPM();

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
        telemetry.addLine("Direction: REVERSE");
        telemetry.addData("Target Limit", "%.0f RPM",
                MAX_TARGET_RPM);
        telemetry.addData("Motor Rated RPM", "%.0f",
                ratedRPM);
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                boolean up1 = gamepad1.dpad_up;
                boolean down1 = gamepad1.dpad_down;
                boolean up2 = gamepad2.dpad_up;
                boolean down2 = gamepad2.dpad_down;

                boolean pressUp1 = up1 && !lastUp1;
                boolean pressDown1 = down1 && !lastDown1;
                boolean pressUp2 = up2 && !lastUp2;
                boolean pressDown2 = down2 && !lastDown2;

                if (gamepad1.a || gamepad2.a) {
                    targetRPM = 0;
                    activeGamepad = gamepad1.a ? 1 : 2;
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
                        targetRPM, 0, MAX_TARGET_RPM
                );

                double targetVelocity =
                        targetRPM * ticksPerRev / 60.0;

                if (targetRPM == 0) {
                    shooter.setPower(0);
                } else {
                    shooter.setVelocity(targetVelocity);
                }

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
                        "MAX TARGET RPM", "%.0f",
                        MAX_TARGET_RPM
                );
                telemetry.addData(
                        "MOTOR RATED RPM", "%.0f",
                        ratedRPM
                );
                telemetry.addData(
                        "RPM STEP", "%.0f", RPM_STEP
                );
                telemetry.addData(
                        "GAMEPAD", activeGamepad
                );

                telemetry.addLine("D-PAD UP: Faster");
                telemetry.addLine("D-PAD DOWN: Slower");
                telemetry.addLine("A: Stop");

                if (targetRPM > ratedRPM) {
                    telemetry.addLine(
                            "WARNING: Above rated motor RPM"
                    );
                }

                telemetry.update();
                sleep(20);
            }
        }
        finally {
            shooter.setPower(0);
        }
    }
}
