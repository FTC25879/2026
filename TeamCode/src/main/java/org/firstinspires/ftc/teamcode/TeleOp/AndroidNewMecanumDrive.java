package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

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
 * HELIX HYDRAULICS #25879 - competition TeleOp.
 * Updated shooter controller: lower integral windup, fast overspeed cutback.
 * Intake X=forward hold, D-pad LEFT=reverse hold; timeout 10 seconds.
 * A=shooter toggle, B=stick toggle, Y=manual gate hold (any RPM).
 * INIT D-pad UP/DOWN changes gate travel; Y previews motion with motor OFF.
 * During match hold RB + D-pad UP/DOWN to trim gate travel (+/-0.010).
 * D-pad UP/DOWN=target +/-10 RPM, LB=diagnostics.
 * Left stick drive/strafe; right stick turn; LT/RT precision.
 * Preserves RUN_WITHOUT_ENCODER and existing hardware names/directions.
 * RPM conversion assumes 28 encoder ticks per motor revolution (1:1).
 */
@TeleOp(name = "25879 - Competition TeleOp Single")
public class AndroidNewMecanumDrive extends LinearOpMode {
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx shooter;
    private DcMotor ballPusher;
    private Servo gate, stick;

    // Shooter speed control. Adjust after measured on-robot testing.
    private static final double DEFAULT_TARGET_RPM = 2650.0;
    private static final double RPM_STEP = 10.0;
    private static final double MIN_TARGET_RPM = 0.0;
    private static final double MAX_TARGET_RPM = 6000.0;
    private static final double TICKS_PER_MOTOR_REV = 28.0;
    private static final double START_POWER = 0.55;
    private static final double NOMINAL_FREE_RPM = 6000.0;
    private static final double SHOOTER_KP = 0.00020;
    private static final double SHOOTER_KI_UP = 0.00006;
    private static final double SHOOTER_KI_DOWN = 0.00025;
    private static final double MAX_POSITIVE_INTEGRAL = 0.08;
    private static final double MAX_NEGATIVE_INTEGRAL = 0.30;
    private static final double RPM_FILTER_ALPHA = 0.55;
    private static final double CONTROL_PERIOD_S = 0.040;
    private static final double STARTUP_POWER_RAMP_PER_S = 0.60;
    private static final double OVERSPEED_COAST_RPM = 130.0;
    private static final double OVERSPEED_RESUME_RPM = 30.0;
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
    private boolean overspeedCoasting = false;
    private boolean rpmInRange = false;
    private boolean rpmReady = false;
    private final ElapsedTime controlTimer = new ElapsedTime();
    private final ElapsedTime spinupTimer = new ElapsedTime();
    private final ElapsedTime stableTimer = new ElapsedTime();

    // GATE POSITION DEFAULTS: set GATE_DEFAULT_OPEN to the desired open position.
    // The gate starts CLOSED (0.189) and opens to 0.309 when Y is held.
    // Legal GATE_DEFAULT_OPEN range with current limits: 0.209 through 0.500.
    private static final double GATE_CLOSED = 0.189;
    private static final double GATE_DEFAULT_OPEN = 0.309;
    private static final double GATE_TRAVEL_STEP = 0.010;
    private static final double GATE_TRAVEL_MIN = 0.020;
    private static final double GATE_TRAVEL_MAX = 0.311;
    // Keep live gamepad tuning as travel offset from the closed position.
    private double gateTravel = Range.clip(GATE_DEFAULT_OPEN - GATE_CLOSED,
            GATE_TRAVEL_MIN, GATE_TRAVEL_MAX);
    private static final double STICK_CLOSED = 0.80;
    private static final double STICK_OPEN = 0.20;
    private boolean stickOpen = false;
    private boolean gatePushing = false;

    private static final double NORMAL_SPEED = 1.0;
    private static final double PRECISION_SPEED = 0.25;
//    private static final double DRIVE_DEADBAND = 0.08;
    private int driveController = 1;
    private boolean precisionDrive = false;
    private boolean precisionTurn = false;

    private enum IntakeMode { OFF, FORWARD, REVERSE }
    private static final double INTAKE_TIMEOUT_S = 10.0;
    private static final double INTAKE_REVERSE_PAUSE_S = 0.15;
    private IntakeMode intakeMode = IntakeMode.OFF;
    private boolean intakeTimedOut = false;
    private boolean intakeRequireRelease = false;
    private boolean intakeReversalPending = false;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime reversalTimer = new ElapsedTime();

    private boolean lastA, lastB, lastUp, lastDown, lastLB;
    private static final double LOW_BATTERY_V = 11.5;
    private static final double CRITICAL_BATTERY_V = 11.0;
    private boolean detailsPage = false;
    private final ElapsedTime screenTimer = new ElapsedTime();
    private final ElapsedTime readyFlashTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        tuneGateBeforeStart();
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

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPower(0);
        ballPusher.setPower(0);
        gate.setPosition(GATE_CLOSED);
        stick.setPosition(STICK_CLOSED);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(200);
    }

    // Runs in INIT, before pressing START. Shooter and intake remain OFF.
    @SuppressLint("DefaultLocale")
    private void tuneGateBeforeStart() {
        boolean previousUp = false;
        boolean previousDown = false;
        while (!isStarted() && !isStopRequested()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (up && !previousUp && !down) changeGateTravel(+1);
            else if (down && !previousDown && !up) changeGateTravel(-1);
            previousUp = up;
            previousDown = down;

            // Hold Y to preview the distance without running the shooter.
            gatePushing = gamepad1.y;
            gate.setPosition(gatePushing ? gateOpenPosition() : GATE_CLOSED);

            telemetry.clearAll();
            telemetry.addLine(color("<b>25879 | GATE SETUP (INIT)</b>", "#55BBFF"));
            telemetry.addLine("D-pad UP = more travel | DOWN = less travel");
            telemetry.addLine("Hold Y = preview gate | release Y = close");
            telemetry.addLine(mono(String.format(
                    "CLOSED %.3f  OPEN %.3f  TRAVEL %.3f",
                    GATE_CLOSED, gateOpenPosition(), gateTravel)));
            telemetry.addLine(mono(String.format(
                    "DEFAULT OPEN %.3f  (editable in Java)", GATE_DEFAULT_OPEN)));
            telemetry.addLine("START = use this setting during the match");
            telemetry.addLine("To reuse next time, edit GATE_DEFAULT_OPEN in Java");
            telemetry.addLine("No balls in mechanism during first motion test");
            telemetry.update();
            sleep(40);
        }
        gatePushing = false;
        gate.setPosition(GATE_CLOSED);
        // Avoid an accidental button edge immediately after START.
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }

    private double gateOpenPosition() {
        return Range.clip(GATE_CLOSED + gateTravel, 0.0, 1.0);
    }

    private void changeGateTravel(int steps) {
        gateTravel = Range.clip(gateTravel + steps * GATE_TRAVEL_STEP,
                GATE_TRAVEL_MIN, GATE_TRAVEL_MAX);
    }

    private void updateButtons() {
        boolean a = gamepad1.a;
        if (a && !lastA) { shooterOn = !shooterOn; resetShooterControl(); }
        lastA = a;

        boolean b = gamepad1.b;
        if (b && !lastB) {
            stickOpen = !stickOpen;
            stick.setPosition(stickOpen ? STICK_OPEN : STICK_CLOSED);
        }
        lastB = b;

        updateIntakeCommand(gamepad1.x, gamepad1.dpad_left);

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        // RB switches D-pad UP/DOWN from shooter RPM to gate travel trim.
        boolean gateTrim = gamepad1.right_bumper;
        if (up && !lastUp && !down) {
            if (gateTrim) changeGateTravel(+1);
            else {
                targetRPM = Math.min(MAX_TARGET_RPM, targetRPM + RPM_STEP);
                resetReadiness();
            }
        } else if (down && !lastDown && !up) {
            if (gateTrim) changeGateTravel(-1);
            else {
                targetRPM = Math.max(MIN_TARGET_RPM, targetRPM - RPM_STEP);
                resetReadiness();
            }
        }
        lastUp = up;
        lastDown = down;

        boolean lb = gamepad1.left_bumper;
        if (lb && !lastLB) detailsPage = !detailsPage;
        lastLB = lb;
    }

    private void updateIntakeCommand(boolean forwardHeld, boolean reverseHeld) {
        if (!forwardHeld && !reverseHeld) {
            intakeMode = IntakeMode.OFF;
            intakeReversalPending = false;
            intakeRequireRelease = false;
            return;
        }
        if (intakeRequireRelease || (forwardHeld && reverseHeld)) {
            intakeMode = IntakeMode.OFF;
            intakeReversalPending = false;
            return;
        }
        IntakeMode requested = forwardHeld ? IntakeMode.FORWARD : IntakeMode.REVERSE;
        if (intakeMode != requested) {
            boolean reversing = intakeMode != IntakeMode.OFF;
            intakeMode = requested;
            intakeTimedOut = false;
            intakeTimer.reset();
            intakeReversalPending = reversing;
            if (reversing) reversalTimer.reset();
        }
    }

    private void resetReadiness() {
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
        overspeedCoasting = false;
        shooterFault = false;
        controlTimer.reset();
        spinupTimer.reset();
        if (!shooterOn) {
            shooterPower = 0;
            shooter.setPower(0);
        } else {
            shooterPower = START_POWER;
            shooter.setPower(shooterPower);
        }
    }

    // Software PI speed control with anti-windup and overspeed coast.
    private void updateShooter() {
        actualRPM = Math.abs(shooter.getVelocity()) * 60.0 / TICKS_PER_MOTOR_REV;

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
            overspeedCoasting = false;
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
        double feedForward = Range.clip(START_POWER
                + (targetRPM - DEFAULT_TARGET_RPM) / NOMINAL_FREE_RPM,
                0.0, 1.0);
        double upperLimit = Range.clip(START_POWER
                + spinupTimer.seconds() * STARTUP_POWER_RAMP_PER_S,
                START_POWER, 1.0);

        if (!overspeedCoasting &&
                (filteredRPM > targetRPM + OVERSPEED_COAST_RPM
                 || actualRPM > targetRPM + 220.0)) {
            overspeedCoasting = true;
            integratedPower = Range.clip(
                    Math.min(integratedPower, 0.0) - 0.03,
                    -MAX_NEGATIVE_INTEGRAL, 0.0);
        }
        if (overspeedCoasting
                && filteredRPM <= targetRPM + OVERSPEED_RESUME_RPM
                && actualRPM <= targetRPM + 80.0) {
            overspeedCoasting = false;
        }
        if (overspeedCoasting) {
            shooterPower = 0.0; // FLOAT: coast; does not actively brake.
            shooter.setPower(0.0);
            return;
        }

        if (error < -50.0) {
            integratedPower = Math.min(integratedPower, 0.0);
        }
        double preliminary = feedForward + SHOOTER_KP * error + integratedPower;
        if (error > 0.0 && error < 180.0 && preliminary < upperLimit) {
            integratedPower += SHOOTER_KI_UP * error * dt;
        } else if (error < 0.0 && preliminary > 0.0) {
            integratedPower += SHOOTER_KI_DOWN * error * dt;
        }
        integratedPower = Range.clip(integratedPower,
                -MAX_NEGATIVE_INTEGRAL, MAX_POSITIVE_INTEGRAL);
        shooterPower = Range.clip(feedForward + SHOOTER_KP * error
                + integratedPower, 0.0, upperLimit);
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
        if (intakeMode != IntakeMode.OFF && intakeTimer.seconds() >= INTAKE_TIMEOUT_S) {
            intakeMode = IntakeMode.OFF;
            intakeTimedOut = true;
            intakeRequireRelease = true;
            intakeReversalPending = false;
        }
        if (intakeReversalPending && reversalTimer.seconds() < INTAKE_REVERSE_PAUSE_S) {
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
        // Gate is deliberately MANUAL: Y while held, no RPM interlock.
        gatePushing = gamepad1.y;
        gate.setPosition(gatePushing ? gateOpenPosition() : GATE_CLOSED);
    }

    private void updateDrive() {
        driveController = 1;
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.1;
        double turn = gamepad1.right_stick_x;
        precisionDrive = gamepad1.left_trigger > 0.1;
        precisionTurn = gamepad1.right_trigger > 0.1;
        applyDrive(forward, strafe, turn);
    }


    private void applyDrive(double forward, double strafe, double turn) {
        double ds = precisionDrive ? PRECISION_SPEED : NORMAL_SPEED;
        double ts = precisionTurn ? PRECISION_SPEED : NORMAL_SPEED;
        double fl = forward * ds + turn * ts + strafe * ds;
        double bl = forward * ds + turn * ts - strafe * ds;
        double fr = forward * ds - turn * ts - strafe * ds;
        double br = forward * ds - turn * ts + strafe * ds;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    @SuppressLint("DefaultLocale")
    private void updateDashboard() {
        if (screenTimer.milliseconds() < 200) return;
        screenTimer.reset();
        double volts = readBatteryVoltage();
        double amps = readShooterAmps();
        double rpmError = actualRPM - targetRPM;
        String state = !shooterOn ? "OFF" : shooterFault ? "ENCODER / SPIN FAULT"
                : overspeedCoasting ? "OVERSPEED - COAST"
                : rpmReady ? "READY"
                : shooterPower >= 0.99 && filteredRPM < targetRPM - RPM_READY_BAND
                  ? "MAX POWER / LOW RPM" : "SPINNING UP / ADJUSTING";
        String stateColor = !shooterOn ? "#9BA9BB" : shooterFault ? "#FF5964"
                : rpmReady ? "#51DF99" : "#FFD166";
        String batteryState = Double.isNaN(volts) ? "N/A"
                : volts < CRITICAL_BATTERY_V ? "CRITICAL"
                : volts < LOW_BATTERY_V ? "LOW" : "OK";
        String batteryColor = Double.isNaN(volts) ? "#9BA9BB"
                : volts < CRITICAL_BATTERY_V ? "#FF5964"
                : volts < LOW_BATTERY_V ? "#FFD166" : "#51DF99";

        telemetry.clearAll();
        telemetry.addLine(color("<b>HELIX HYDRAULICS | #25879</b>", "#55BBFF"));
        if (!detailsPage) {
            if (rpmReady) {
                boolean bright = ((int) (readyFlashTimer.milliseconds() / 500)) % 2 == 0;
                telemetry.addLine(color("<big><big><b>*** READY TO SHOOT ***</b></big></big>",
                        bright ? "#00FF55" : "#197843"));
            } else if (shooterFault) {
                telemetry.addLine(color("<big><b>!!! SHOOTER FAULT !!!</b></big>", "#FF5964"));
            } else if (overspeedCoasting) {
                telemetry.addLine(color("<big><b>OVERSPEED - COAST</b></big>", "#FFD166"));
            } else if (shooterOn) {
                telemetry.addLine(color("<big><b>NOT READY - WAIT FOR RPM</b></big>", "#FFD166"));
            } else {
                telemetry.addLine(color("<big><b>SHOOTER OFF</b></big>", "#9BA9BB"));
            }
            telemetry.addLine(mono(String.format("RPM ACT %.0f  TARGET %.0f  ERR %+.0f",
                    actualRPM, targetRPM, rpmError)));
            telemetry.addLine(color(mono("RPM " + gauge(filteredRPM, targetRPM, 14)),
                    rpmReady ? "#51DF99" : "#55BBFF"));
            telemetry.addLine(mono(String.format("PWR %.1f%%  I %sA  BAT %sV  ",
                    shooterPower * 100.0, fmt(amps, 1), fmt(volts, 1)))
                    + color(batteryState, batteryColor));
            telemetry.addLine(color("<b>INTAKE " + intakeStatusText()
                    + " | TIME LEFT " + intakeCountdownText() + "</b>",
                    intakeTimedOut ? "#FFD166"
                    : intakeMode == IntakeMode.OFF ? "#9BA9BB" : "#51DF99"));
            telemetry.addLine("GATE " + status(gatePushing, "PUSH", "CLOSED")
                    + "  STICK " + status(stickOpen, "OPEN", "CLOSED"));
            telemetry.addLine(mono(String.format("GATE OPEN %.3f | TRAVEL %.3f",
                    gateOpenPosition(), gateTravel)));
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
            telemetry.addLine("Stable +/-20 RPM for 350ms: " + status(rpmReady, "YES", "NO"));
            telemetry.addLine(mono(String.format("Power: %.2f%%  PI I-term: %+.4f",
                    shooterPower * 100.0, integratedPower)));
            telemetry.addLine(mono("Current: " + fmt(amps, 2) + " A"));
            telemetry.addLine(color("Battery: " + fmt(volts, 2)
                    + " V " + batteryState, batteryColor));
            telemetry.addLine(mono(String.format("Gate: %.3f  Stick: %.3f",
                    gate.getPosition(), stick.getPosition())));
            telemetry.addLine(mono(String.format("Gate closed %.3f | open %.3f | travel %.3f",
                    GATE_CLOSED, gateOpenPosition(), gateTravel)));
            telemetry.addLine("Intake: " + intakeStatusText()
                    + " | Time left: " + intakeCountdownText());
            telemetry.addLine("Shooter " + status(shooterOn, "ON", "OFF"));
            telemetry.addLine("Encoder: 28 ticks/motor rev | 1:1 assumed");
            if (shooterFault) telemetry.addLine("SHOOTER FAULT: CHECK ENCODER / MOTOR");
            telemetry.addLine("LB: back to MATCH screen");
        }
        telemetry.update();
    }

    private String intakeStatusText() {
        if (intakeTimedOut) return "AUTO OFF (10s)";
        if (intakeMode == IntakeMode.FORWARD) return "FORWARD";
        if (intakeMode == IntakeMode.REVERSE) return "REVERSE";
        return "OFF";
    }

    @SuppressLint("DefaultLocale")
    private String intakeCountdownText() {
        if (intakeTimedOut) return "0.0s";
        if (intakeMode == IntakeMode.OFF) return "--";
        return String.format("%.1fs", Math.max(0.0,
                INTAKE_TIMEOUT_S - intakeTimer.seconds()));
    }

    private void addControlGuide() {
        telemetry.addLine(color("<b>GAMEPAD 1 - ALL COMMANDS</b>", "#55BBFF"));
        telemetry.addLine("L STICK MOVE/STRAFE | R STICK TURN | LT/RT SLOW");
        telemetry.addLine("A SHOOT TOGGLE | B STICK TOGGLE");
        telemetry.addLine("X FWD INTAKE HOLD | DPAD LEFT REV HOLD");
        telemetry.addLine("Y GATE HOLD (MANUAL, ANY RPM)");
        telemetry.addLine("DPAD UP +10 / DOWN -10 RPM | LB DETAILS");
        telemetry.addLine("HOLD RB + DPAD UP/DOWN = GATE TRAVEL +/-0.010");
        telemetry.addLine("INTAKE MAX 10s | RELEASE BEFORE RESTART");
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
