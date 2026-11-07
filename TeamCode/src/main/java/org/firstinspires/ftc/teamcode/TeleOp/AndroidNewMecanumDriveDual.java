package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * HELIX HYDRAULICS #25879 - full mecanum TeleOp / Driver Hub dashboard.
 *
 * All controls are shown on the main Driver Hub telemetry page.
 * A: shooter ON/OFF toggle                 B: stick OPEN/CLOSED toggle
 * X: forward intake ON/OFF toggle         Y: gate push (hold, RPM-ready only)
 * BOTH GAMEPADS: identical controls. Gamepad 1 has priority for driving
 * when its joysticks are active; otherwise Gamepad 2 can drive.
 * D-pad LEFT: hold for reverse intake (release stops it)
 * D-pad UP/DOWN: shooter target +/-10 REAL RPM (feedback control)
 * D-pad RIGHT: unassigned                 LB: match/diagnostics pages
 * Intake runs for a maximum of 10 seconds per activation (either direction).
 * Left stick: drive/strafe                 Right stick: turn
 * LT: precision drive                     RT: precision turn
 *
 * Uses the EXISTING WORKING RUN_WITHOUT_ENCODER shooter mode, adding
 * software PI feedback via DcMotorEx.getVelocity(). Requires a working
 * encoder cable and a bare HD Hex motor with 28 counts/motor revolution.
 * Motor power is limited to 100%; an unreachable RPM cannot be guaranteed.
 */
@TeleOp(name = "Android New Mecanum Drive Dual")
public class AndroidNewMecanumDriveDual extends LinearOpMode {

    // ---------- Hardware: names must match Robot Configuration ----------
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx shooter;
    private DcMotor ballPusher;
    private Servo gate, stick;

    // ---------- Shooter: feedback control starting from the working 55% ----------
    private static final double DEFAULT_TARGET_RPM = 2650.0;
    private static final double RPM_STEP = 10.0;
    private static final double MIN_TARGET_RPM = 0.0;
    private static final double MAX_TARGET_RPM = 6000.0;
    private static final double TICKS_PER_MOTOR_REV = 28.0;
    private static final double START_POWER = 0.55;
    private static final double NOMINAL_FREE_RPM = 6000.0;

    // Initial PI gains (power per RPM; I is power per RPM-second).
    // Adjust only after watching the measured response; tuning is robot-specific.
    private static final double SHOOTER_KP = 0.00017;
    private static final double SHOOTER_KI = 0.00030;
    private static final double MAX_INTEGRAL = 0.55;
    private static final double RPM_FILTER_ALPHA = 0.40;
    private static final double CONTROL_PERIOD_S = 0.040;
    private static final double STARTUP_POWER_RAMP_PER_S = 0.60;

    private static final double RPM_READY_BAND = 20.0;
    private static final double RPM_RAW_BAND = 50.0;
    private static final double RPM_READY_TIME_MS = 350.0;
    private double targetRPM = DEFAULT_TARGET_RPM;
    private double actualRPM = 0;
    private double filteredRPM = 0;
    private double shooterPower = 0;
    private double integratedPower = 0;
    private boolean filterInitialized = false;
    private boolean shooterOn = false;
    private boolean shooterFault = false;
    private boolean rpmInRange = false;
    private boolean rpmReady = false;
    private final ElapsedTime controlTimer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();

    // ---------- Servos ----------
    private static final double GATE_CLOSED = 0.189;
    private static final double GATE_OPEN = 0.50;
    // INVERTED from earlier .20 closed / .80 open (physical stick reversal).
    private static final double STICK_CLOSED = 0.80;
    private static final double STICK_OPEN = 0.20;
    private boolean stickOpen = false; // Start physically CLOSED.
    private boolean gatePushing = false;

    // ---------- Drive ----------
    private static final double NORMAL_SPEED = 1.0;
    private static final double PRECISION_SPEED = 0.25;
    private static final double DRIVE_DEADBAND = 0.08;
    private int driveController = 1;
    private boolean precisionDrive = false;
    private boolean precisionTurn = false;

    // ---------- Intake: direction selection with 10-second auto-stop ----------
    private enum IntakeMode { OFF, FORWARD, REVERSE }
    private static final double INTAKE_TIMEOUT_S = 10.0;
    private static final double INTAKE_REVERSE_PAUSE_S = 0.15;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private boolean intakeTimedOut = false;
    private boolean intakeReversalPending = false;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime reversalTimer = new ElapsedTime();

    // ---------- Independent button edges for BOTH gamepads ----------
    private boolean lastA1, lastA2, lastB1, lastB2, lastX1, lastX2;
    private boolean lastY1, lastY2, lastUp1, lastUp2, lastDown1, lastDown2;
    private boolean lastLeft, lastLB1, lastLB2;

    // ---------- Driver Station ----------
    private static final double LOW_BATTERY_V = 11.5;
    private static final double CRITICAL_BATTERY_V = 11.0;
    private boolean detailsPage = false;
    private final ElapsedTime screenTimer = new ElapsedTime();
    private final ElapsedTime readyFlashTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        showInitScreen();
        waitForStart();
        if (isStopRequested()) return;

        controlTimer.reset();
        screenTimer.reset();
        readyFlashTimer.reset();
        try {
            while (opModeIsActive()) {
                updateButtons();
                updateShooter();
                updateShooterStatus();
                updateIntakeAndServos();
                updateDrive();
                updateDashboard();
                idle();
            }
        } finally {
            stopAllMotors();
            gate.setPosition(GATE_CLOSED);
        }
    }

    private void initializeHardware() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ballPusher = hardwareMap.get(DcMotor.class, "ballPusher");
        gate = hardwareMap.get(Servo.class, "gate");
        stick = hardwareMap.get(Servo.class, "stick");

        // Preserve working drivetrain motor directions.
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // Preserve the working power mode. Encoder feedback is read separately.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPower(0);
        ballPusher.setPower(0);
        gate.setPosition(GATE_CLOSED);
        stick.setPosition(STICK_CLOSED);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(200);
    }

    private void showInitScreen() {
        telemetry.clearAll();
        telemetry.addLine(color("<b>HELIX HYDRAULICS | #25879</b>", "#55BBFF"));
        telemetry.addLine("<b>NEW MECANUM DRIVE - READY</b>");
        addControlGuide();
        telemetry.addLine(color("Press START on Driver Hub", "#51DF99"));
        telemetry.update();
    }

    private void updateButtons() {
        // Edge detection is per gamepad: either driver can issue a fresh press.
        // Simultaneous presses of the same button count as one action per loop.
        boolean aPressed = (gamepad1.a && !lastA1) || (gamepad2.a && !lastA2);
        if (aPressed) {
            shooterOn = !shooterOn;
            resetShooterControl();
        }
        lastA1 = gamepad1.a;
        lastA2 = gamepad2.a;

        boolean bPressed = (gamepad1.b && !lastB1) || (gamepad2.b && !lastB2);
        if (bPressed) {
            stickOpen = !stickOpen;
            stick.setPosition(stickOpen ? STICK_OPEN : STICK_CLOSED);
        }
        lastB1 = gamepad1.b;
        lastB2 = gamepad2.b;

        // X: toggle forward intake from EITHER gamepad.
        // D-pad LEFT: hold to reverse. Release of both buttons stops reverse.
        // Reverse never auto-restarts if the 10-second timer expires while held.
        boolean xPressed = (gamepad1.x && !lastX1) || (gamepad2.x && !lastX2);
        boolean reverseHeld = gamepad1.dpad_left || gamepad2.dpad_left;

        if (reverseHeld && !lastLeft) {
            selectIntake(IntakeMode.REVERSE);
        } else if (!reverseHeld && lastLeft && intakeMode == IntakeMode.REVERSE) {
            intakeMode = IntakeMode.OFF;
            intakeReversalPending = false;
        }
        if (xPressed && !reverseHeld) {
            toggleIntake(IntakeMode.FORWARD);
        }
        lastX1 = gamepad1.x;
        lastX2 = gamepad2.x;
        lastLeft = reverseHeld;

        boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean upPressed = (gamepad1.dpad_up && !lastUp1)
                || (gamepad2.dpad_up && !lastUp2);
        boolean downPressed = (gamepad1.dpad_down && !lastDown1)
                || (gamepad2.dpad_down && !lastDown2);
        if (upPressed && !down) {
            targetRPM = Math.min(MAX_TARGET_RPM, targetRPM + RPM_STEP);
            resetReadiness();
        } else if (downPressed && !up) {
            targetRPM = Math.max(MIN_TARGET_RPM, targetRPM - RPM_STEP);
            resetReadiness();
        }
        lastUp1 = gamepad1.dpad_up;
        lastUp2 = gamepad2.dpad_up;
        lastDown1 = gamepad1.dpad_down;
        lastDown2 = gamepad2.dpad_down;

        boolean lbPressed = (gamepad1.left_bumper && !lastLB1)
                || (gamepad2.left_bumper && !lastLB2);
        if (lbPressed) detailsPage = !detailsPage;
        lastLB1 = gamepad1.left_bumper;
        lastLB2 = gamepad2.left_bumper;
    }

    private void selectIntake(IntakeMode selectedMode) {
        if (intakeMode != selectedMode) {
            intakeReversalPending = intakeMode != IntakeMode.OFF;
            if (intakeReversalPending) reversalTimer.reset();
            intakeMode = selectedMode;
            intakeTimer.reset();
        }
        intakeTimedOut = false;
    }

    private void toggleIntake(IntakeMode selectedMode) {
        if (intakeMode == selectedMode) {
            // Pressing X again stops the forward intake.
            intakeMode = IntakeMode.OFF;
            intakeReversalPending = false;
        } else {
            // Avoid instantly changing a moving motor from full forward
            // to full reverse (or vice versa): coast at zero for 150 ms.
            intakeReversalPending = intakeMode != IntakeMode.OFF;
            if (intakeReversalPending) reversalTimer.reset();
            intakeMode = selectedMode;
            intakeTimer.reset();
        }
        intakeTimedOut = false;
    }

    private String intakeStatusText() {
        if (intakeTimedOut) return "AUTO OFF (10s)";
        if (intakeMode == IntakeMode.FORWARD) return "FORWARD";
        if (intakeMode == IntakeMode.REVERSE) return "REVERSE";
        return "OFF";
    }

    private String intakeCountdownText() {
        if (intakeTimedOut) return "0.0s";
        if (intakeMode == IntakeMode.OFF) return "--";
        return String.format("%.1fs", Math.max(0.0,
                INTAKE_TIMEOUT_S - intakeTimer.seconds()));
    }

    private void resetReadiness() {
        // A new shooter speed invalidates any gate authorization.
        gatePushing = false;
        if (gate != null) gate.setPosition(GATE_CLOSED);
        rpmInRange = false;
        rpmReady = false;
        stableTimer.reset();
    }

    private void resetShooterControl() {
        resetReadiness();
        filterInitialized = false;
        integratedPower = 0;
        shooterFault = false;
        controlTimer.reset();
        spinupTimer.reset();
        if (!shooterOn) {
            shooter.setPower(0);
            shooterPower = 0;
        } else {
            shooterPower = START_POWER;
            shooter.setPower(shooterPower);
        }
    }

    /** Software PI controller. The I term removes persistent offset. */
    private void updateShooter() {
        // Get real encoder speed even in RUN_WITHOUT_ENCODER mode.
        actualRPM = Math.abs(shooter.getVelocity()) * 60.0
                / TICKS_PER_MOTOR_REV;

        // Missing encoder feedback must not command maximum motor power.
        // If the motor has had time to spin but reports virtually no motion,
        // stop it. Switch A OFF then ON to retry after fixing the issue.
        if (shooterOn && !shooterFault && spinupTimer.seconds() > 1.5
                && actualRPM < 100.0 && shooterPower >= 0.50) {
            shooterFault = true;
            shooter.setPower(0);
            shooterPower = 0;
            resetReadiness();
        }

        if (!shooterOn || shooterFault || targetRPM <= 0) {
            shooterPower = 0;
            integratedPower = 0;
            shooter.setPower(0);
            return;
        }

        double elapsed = controlTimer.seconds();
        if (elapsed < CONTROL_PERIOD_S) return;
        controlTimer.reset();
        double dt = Range.clip(elapsed, 0.01, 0.15);

        if (!filterInitialized) {
            filteredRPM = actualRPM;
            filterInitialized = true;
        } else {
            filteredRPM += RPM_FILTER_ALPHA * (actualRPM - filteredRPM);
        }

        double error = targetRPM - filteredRPM;
        double feedForward = START_POWER
                + (targetRPM - DEFAULT_TARGET_RPM) / NOMINAL_FREE_RPM;
        feedForward = Range.clip(feedForward, 0.0, 1.0);

        // Limit aggressive startup surges from zero speed; increase toward
        // full available power as the flywheel spins up.
        double startupLimit = Range.clip(
                START_POWER + spinupTimer.seconds() * STARTUP_POWER_RAMP_PER_S,
                START_POWER, 1.0);
        double preliminary = feedForward + SHOOTER_KP * error + integratedPower;
        double upperLimit = Math.min(1.0, startupLimit);

        // Anti-windup: do not integrate farther into a saturated output.
        boolean atUpperLimit = preliminary >= upperLimit && error > 0;
        boolean atLowerLimit = preliminary <= 0.0 && error < 0;
        if (!atUpperLimit && !atLowerLimit) {
            integratedPower = Range.clip(integratedPower
                            + SHOOTER_KI * error * dt,
                    -MAX_INTEGRAL, MAX_INTEGRAL);
        }

        shooterPower = Range.clip(
                feedForward + SHOOTER_KP * error + integratedPower,
                0.0, upperLimit);
        shooter.setPower(shooterPower);
    }

    private void updateShooterStatus() {
        if (!shooterOn || shooterFault || targetRPM <= 0) {
            resetReadiness();
            return;
        }
        boolean inBandNow = Math.abs(filteredRPM - targetRPM) <= RPM_READY_BAND
                && Math.abs(actualRPM - targetRPM) <= RPM_RAW_BAND;
        if (inBandNow && !rpmInRange) stableTimer.reset();
        rpmInRange = inBandNow;
        rpmReady = inBandNow && stableTimer.milliseconds() >= RPM_READY_TIME_MS;
    }

    private void updateIntakeAndServos() {
        // Enforce a 10-second maximum continuous intake run in BOTH directions.
        if (intakeMode != IntakeMode.OFF
                && intakeTimer.seconds() >= INTAKE_TIMEOUT_S) {
            intakeMode = IntakeMode.OFF;
            intakeTimedOut = true;
            intakeReversalPending = false;
        }
        if (intakeReversalPending
                && reversalTimer.seconds() < INTAKE_REVERSE_PAUSE_S) {
            ballPusher.setPower(0.0);
        } else if (intakeMode == IntakeMode.FORWARD) {
            intakeReversalPending = false;
            ballPusher.setPower(1.0);
        } else if (intakeMode == IntakeMode.REVERSE) {
            intakeReversalPending = false;
            ballPusher.setPower(-1.0);
        } else {
            ballPusher.setPower(0.0);
        }


        // =====================================
        // GATE - MANUAL CONTROL, NO RPM CHECK
        // =====================================

        // Either gamepad can operate the gate
                boolean y = gamepad1.y || gamepad2.y;

        // Open while Y is held, close when released
                gatePushing = y;

                gate.setPosition(
                        gatePushing ? GATE_OPEN : GATE_CLOSED
                );
    }

    private void updateDrive() {
        // Priority: Gamepad 1 if its sticks are moved outside the deadband.
        // Otherwise Gamepad 2 drives. Never sum opposing joystick commands.
        double p1Activity = Math.max(Math.abs(gamepad1.left_stick_y),
                Math.max(Math.abs(gamepad1.left_stick_x),
                        Math.abs(gamepad1.right_stick_x)));
        double p2Activity = Math.max(Math.abs(gamepad2.left_stick_y),
                Math.max(Math.abs(gamepad2.left_stick_x),
                        Math.abs(gamepad2.right_stick_x)));
        boolean usePad2 = p1Activity <= DRIVE_DEADBAND
                && p2Activity > DRIVE_DEADBAND;
        driveController = usePad2 ? 2 : 1;

        double forward = -(usePad2 ? gamepad2.left_stick_y : gamepad1.left_stick_y);
        double strafe = (usePad2 ? gamepad2.left_stick_x : gamepad1.left_stick_x) * 1.1;
        double turn = usePad2 ? gamepad2.right_stick_x : gamepad1.right_stick_x;
        if (Math.abs(forward) < DRIVE_DEADBAND) forward = 0;
        if (Math.abs(strafe) < DRIVE_DEADBAND) strafe = 0;
        if (Math.abs(turn) < DRIVE_DEADBAND) turn = 0;

        // Either driver can request precision mode; do not multiply inputs.
        precisionDrive = gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1;
        precisionTurn = gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1;
        double driveScale = precisionDrive ? PRECISION_SPEED : NORMAL_SPEED;
        double turnScale = precisionTurn ? PRECISION_SPEED : NORMAL_SPEED;

        double fl = forward * driveScale + turn * turnScale + strafe * driveScale;
        double bl = forward * driveScale + turn * turnScale - strafe * driveScale;
        double fr = forward * driveScale - turn * turnScale - strafe * driveScale;
        double br = forward * driveScale - turn * turnScale + strafe * driveScale;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    private void updateDashboard() {
        if (screenTimer.milliseconds() < 200) return;
        screenTimer.reset();
        double volts = readBatteryVoltage();
        double amps = readShooterAmps();
        double error = actualRPM - targetRPM;
        String state = !shooterOn ? "OFF" : shooterFault ? "ENCODER / SPIN FAULT"
                                            : rpmReady ? "READY"
                                              : shooterPower >= 0.99 && filteredRPM < targetRPM - RPM_READY_BAND
                                                ? "MAX POWER / LOW RPM" : "SPINNING UP / ADJUSTING";
        String stateColor = !shooterOn ? "#9BA9BB" : shooterFault ? "#FF5964"
                                                     : rpmReady ? "#51DF99"
                                                       : shooterPower >= 0.99 ? "#FF5964" : "#FFD166";
        String batteryState = Double.isNaN(volts) ? "N/A"
                : volts < CRITICAL_BATTERY_V ? "CRITICAL"
                  : volts < LOW_BATTERY_V ? "LOW" : "OK";
        String batteryColor = Double.isNaN(volts) ? "#9BA9BB"
                : volts < CRITICAL_BATTERY_V ? "#FF5964"
                  : volts < LOW_BATTERY_V ? "#FFD166" : "#51DF99";

        telemetry.clearAll();
        telemetry.addLine(color("<b>HELIX HYDRAULICS | #25879</b>", "#55BBFF"));
        if (!detailsPage) {
            // Flash bright / dark green while READY; refresh ~5x per second.
            // A large, always-visible status line remains readable while dark.
            if (rpmReady) {
                boolean bright = ((int) (readyFlashTimer.milliseconds() / 500)) % 2 == 0;
                telemetry.addLine(color("<big><big><b>*** READY TO SHOOT ***</b></big></big>",
                        bright ? "#00FF55" : "#197843"));
            } else if (shooterFault) {
                telemetry.addLine(color("<big><b>!!! SHOOTER FAULT !!!</b></big>", "#FF5964"));
            } else if (shooterOn) {
                telemetry.addLine(color("<big><b>NOT READY - WAIT FOR RPM</b></big>", "#FFD166"));
            } else {
                telemetry.addLine(color("<big><b>SHOOTER OFF</b></big>", "#9BA9BB"));
            }
            telemetry.addLine(mono(String.format("RPM ACT %.0f  TARGET %.0f  ERR %+.0f",
                    actualRPM, targetRPM, error)));
            telemetry.addLine(color(mono("RPM " + gauge(filteredRPM, targetRPM, 14)),
                    rpmReady ? "#51DF99" : "#55BBFF"));
            telemetry.addLine(mono(String.format("PWR %.1f%%  I %sA  BAT %sV  ",
                    shooterPower * 100.0, fmt(amps, 1), fmt(volts, 1)))
                    + color(batteryState, batteryColor));
            telemetry.addLine(color("<b>INTAKE " + intakeStatusText()
                            + " | TIME LEFT " + intakeCountdownText() + "</b>",
                    intakeTimedOut ? "#FFD166" :
                            intakeMode == IntakeMode.OFF ? "#9BA9BB" : "#51DF99"));
            telemetry.addLine("GATE " + status(gatePushing, "PUSH", "CLOSED")
                    + "  STICK " + status(stickOpen, "OPEN", "CLOSED"));
            telemetry.addLine("DRIVER GP" + driveController
                    + "  DRIVE " + (precisionDrive ? "25%" : "100%")
                    + "  TURN " + (precisionTurn ? "25%" : "100%"));
            addControlGuide();
        } else {
            telemetry.addLine(color("<b>DIAGNOSTICS | LB: MATCH</b>", "#FFD166"));
            telemetry.addLine("Shooter: " + color(state, stateColor));
            telemetry.addLine(mono(String.format("Target: %.0f RPM  Raw: %.1f RPM",
                    targetRPM, actualRPM)));
            telemetry.addLine(mono(String.format("Filtered: %.1f  Error: %+.1f RPM",
                    filteredRPM, targetRPM - filteredRPM)));
            telemetry.addLine("Stable +/-20 RPM for 350ms: "
                    + status(rpmReady, "YES", "NO"));
            telemetry.addLine(mono(String.format("Power: %.2f%%  PI I-term: %+.4f",
                    shooterPower * 100.0, integratedPower)));
            telemetry.addLine(mono("Current: " + fmt(amps, 2) + " A"));
            telemetry.addLine(color("Battery: " + fmt(volts, 2)
                    + " V " + batteryState, batteryColor));
            telemetry.addLine(mono(String.format("Gate: %.3f  Stick: %.3f",
                    gate.getPosition(), stick.getPosition())));
            telemetry.addLine("Intake: " + intakeStatusText()
                    + " | Time left: " + intakeCountdownText());
            telemetry.addLine("Shooter " + status(shooterOn, "ON", "OFF"));
            telemetry.addLine("Encoder: 28 ticks/motor rev | 1:1 assumed");
            if (shooterFault) telemetry.addLine("SHOOTER FAULT: CHECK ENCODER / MOTOR");
            telemetry.addLine("LB: back to MATCH screen | GP1/GP2 same commands");
        }
        telemetry.update();
    }

    private void addControlGuide() {
        telemetry.addLine(color("<b>GAMEPADS 1 &amp; 2 - SAME COMMANDS</b>", "#55BBFF"));
        telemetry.addLine("L STICK MOVE/STRAFE | R STICK TURN | LT/RT SLOW");
        telemetry.addLine("A SHOOT TOGGLE  | B STICK TOGGLE");
        telemetry.addLine("X INTAKE FWD TOGGLE | DPAD LEFT REV HOLD");
        telemetry.addLine("Y GATE HOLD (RPM READY ONLY)");
        telemetry.addLine("DPAD UP +10 / DOWN -10 RPM | RIGHT UNUSED");
        telemetry.addLine("INTAKE 10s AUTO OFF | LB DETAILS | GP1 DRIVE PRIORITY");
    }

    private String status(boolean enabled, String yes, String no) {
        return color(enabled ? yes : no, enabled ? "#51DF99" : "#9BA9BB");
    }

    private String gauge(double actual, double target, int width) {
        double fraction = target > 0 ? actual / target : 0;
        int filled = (int) Math.round(Range.clip(fraction, 0, 1) * width);
        StringBuilder s = new StringBuilder("[");
        for (int i = 0; i < width; i++) s.append(i < filled ? "#" : "-");
        return s.append("]").toString();
    }

    private String color(String content, String hex) {
        return "<font color='" + hex + "'>" + content + "</font>";
    }

    private String mono(String content) {
        return "<font face='monospace'>" + content + "</font>";
    }

    private String fmt(double value, int digits) {
        if (Double.isNaN(value)) return "--";
        return String.format(digits == 1 ? "%.1f" : "%.2f", value);
    }

    private double readBatteryVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (volts > 0 && volts < lowest) lowest = volts;
        }
        return lowest == Double.POSITIVE_INFINITY ? Double.NaN : lowest;
    }

    private double readShooterAmps() {
        try {
            return shooter.getCurrent(CurrentUnit.AMPS);
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        shooter.setPower(0);
        ballPusher.setPower(0);
    }
}