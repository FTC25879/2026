
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Android Intake Control")
public class AndroidAdjustableIntake extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor intake = hardwareMap.get(
                DcMotor.class, "ballPusher"
        );

        double power = 0.0;
        int activeGamepad = 0;

        boolean lastUp1 = false;
        boolean lastDown1 = false;
        boolean lastUp2 = false;
        boolean lastDown2 = false;

        intake.setPower(0);

        telemetry.addLine("Intake Ready");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Read BOTH gamepads
                boolean up1 = gamepad1.dpad_up;
                boolean down1 = gamepad1.dpad_down;
                boolean up2 = gamepad2.dpad_up;
                boolean down2 = gamepad2.dpad_down;

                // Detect new button presses
                boolean pressUp1 = up1 && !lastUp1;
                boolean pressDown1 = down1 && !lastDown1;
                boolean pressUp2 = up2 && !lastUp2;
                boolean pressDown2 = down2 && !lastDown2;

                // Any gamepad can stop the intake
                if (gamepad1.a || gamepad2.a) {
                    power = 0.0;
                }
                else if (pressUp1 || pressDown1) {
                    activeGamepad = 1;
                    if (pressUp1) power += 0.05;
                    if (pressDown1) power -= 0.05;
                }
                else if (pressUp2 || pressDown2) {
                    activeGamepad = 2;
                    if (pressUp2) power += 0.05;
                    if (pressDown2) power -= 0.05;
                }

                lastUp1 = up1;
                lastDown1 = down1;
                lastUp2 = up2;
                lastDown2 = down2;

                power = Range.clip(power, 0.0, 1.0);

                // Control ONLY the intake
                intake.setPower(power);

                // Driver Hub telemetry
                telemetry.addData(
                        "INTAKE POWER", "%.0f%%", power * 100
                );
                telemetry.addData(
                        "Last Gamepad", activeGamepad
                );
                telemetry.addData(
                        "Gamepad 1 UP", up1
                );
                telemetry.addData(
                        "Gamepad 2 UP", up2
                );
                telemetry.addData(
                        "Gamepad 1 DOWN", down1
                );
                telemetry.addData(
                        "Gamepad 2 DOWN", down2
                );
                telemetry.update();

                sleep(20);
            }
        }
        finally {
            intake.setPower(0);
        }
    }
}
