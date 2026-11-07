package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * HELIX #25879 - autonomous WITH SHOOTER.
 * Order: shooter ON -> forward firstDistance -> five gate cycles (<=3s RPM wait
 * EACH) -> shooter OFF -> strafe LEFT 26 -> forward (52 + 24 - firstDistance)
 * -> strafe LEFT 20 -> stop every motor.
 *
 * No driver input is read after START. Move distances rely on working drive
 * motor encoders and calibrated drive/strafe ticks-per-inch constants.
 */
@Autonomous(name = "Auto Shooter", group = "Helix")
public class AutoShooter extends LinearOpMode {

    // ---------- Driver-adjustable during INIT ----------
    private static final double DEFAULT_FIRST_FORWARD_IN = 24.0;
    private static final double DEFAULT_SHOOTER_RPM = 2650.0;
    private double baseDistanceIn = 58.0;
    private double firstForwardIn = DEFAULT_FIRST_FORWARD_IN;
    private double targetRPM = DEFAULT_SHOOTER_RPM;
    private boolean lastUp, lastDown, lastLeft, lastRight, lastA, lastB;

    private static final long INTAKE_FEED_MS = 1000;
    private static final double INTAKE_FEED_POWER = 1.0;

    // ---------- Field path ----------
    private static final double FIRST_DRIVE_POWER = 0.55;
    private static final double REPOSITION_POWER = 0.60;
    private static final double LEFT_FIRST_IN = 26.0;
    private static final double LEFT_LAST_IN = 25.0;

    // EXAMPLE: 20:1 REV HD Hex drive motor (560 output ticks/rev), 96mm wheels.
    // Verify motor gearing and actual wheel size; then calibrate on the floor.
    private static final double DRIVE_TICKS_PER_INCH = 560.0 / (Math.PI * (96.0 / 25.4));
    // Mecanum strafe loses distance due to sliding; adjust from measured travel.
    private static final double STRAFE_MULTIPLIER = 1.15;
    private static final int POSITION_TOLERANCE_TICKS = 25;

    // ---------- Shooter: same working PI approach as NewMecanumDrive ----------
    private static final double TICKS_PER_SHOOTER_REV = 28.0; // bare HD Hex, 1:1
    private static final double START_POWER = 0.55;
    private static final double NOMINAL_FREE_RPM = 6000.0;
    private static final double SHOOTER_KP = 0.00017;
    private static final double SHOOTER_KI = 0.00030;
    private static final double MAX_INTEGRAL = 0.55;
    private static final double RPM_FILTER_ALPHA = 0.40;
    private static final double SHOOTER_CONTROL_PERIOD = 0.040;
    private static final double STARTUP_RAMP_PER_SECOND = 0.60;
    private static final double READY_FILTERED_BAND_RPM = 20.0;
    private static final double READY_RAW_BAND_RPM = 50.0;
    private static final long STABLE_MS = 350;
    private static final double MAX_RPM_WAIT_SEC = 3.0; // per ball, NOT total
    private static final int SHOTS = 4;
    private static final long GATE_OPEN_MS = 250;
    private static final long GATE_CLOSE_MS = 250;
    private static final double GATE_CLOSED = 0.189;
    private static final double GATE_OPEN = 0.50;

    // AUTO period is 30s. Stop all powered activity with safety margin.
    private static final double AUTO_LIMIT_SEC = 29.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooter;
    private DcMotor ballPusher;
    private Servo gate;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime shooterControlTimer = new ElapsedTime();
    private final ElapsedTime shooterSpinTimer = new ElapsedTime();
    private double actualRPM = 0;
    private double filteredRPM = 0;
    private double shooterPower = 0;
    private double integratedPower = 0;
    private boolean filterInitialized = false;
    private boolean shooterEnabled = false;
    private boolean shooterFault = false;
    private int gateCycles = 0;

    @Override
    public void runOpMode() {
        initializeHardware();

        // All selections are BEFORE START, never during AUTO.
        while (opModeInInit() && !isStopRequested()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            if (up && !lastUp && !down) targetRPM = Range.clip(targetRPM + 10, 500, 5000);
            if (down && !lastDown && !up) targetRPM = Range.clip(targetRPM - 10, 500, 5000);
            if (right && !lastRight && !left) firstForwardIn = Range.clip(firstForwardIn + 1, 0, 60);
            if (left && !lastLeft && !right) firstForwardIn = Range.clip(firstForwardIn - 1, 0, 60);
            lastUp = up;
            lastDown = down;
            lastLeft = left;
            lastRight = right;
            // A = decrease base distance by 1 inch
            // B = increase base distance by 1 inch
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;

            if (a && !lastA && !b) {
                baseDistanceIn = Range.clip(baseDistanceIn - 1, 0, 120);
            }

            if (b && !lastB && !a) {
                baseDistanceIn = Range.clip(baseDistanceIn + 1, 0, 120);
            }

            lastA = a;
            lastB = b;

            telemetry.addLine("HELIX #25879  |  AUTO - SHOOTER");
            telemetry.addData("1. Drive to shooter", "%.0f inches", firstForwardIn);
            telemetry.addData("2. Drive after shooting", "%.0f inches", remainingForward());
            telemetry.addLine("A/B: Base distance -1/+1 inch");
            telemetry.addData("Shooter target", "%.0f RPM", targetRPM);
            telemetry.addLine("DPAD LEFT/RIGHT: first forward -1/+1 inch");
            telemetry.addLine("DPAD DOWN/UP: shooter RPM -10/+10");
            telemetry.addLine("Path: forward, shots with intake activation, left, forward, left");
            telemetry.addLine("Check motor encoder connections before START");
            telemetry.update();
            sleep(30);
        }

        waitForStart();
        if (isStopRequested()) return;
        matchTimer.reset();
        String outcome = "COMPLETED";
        try {
            startShooter();

            // Drive toward shooting spot WHILE continuously controlling RPM.
            if (!move( firstForwardIn, 0, FIRST_DRIVE_POWER, "Forward to shoot")) {
                outcome = "ABORT: first drive";
                return;
            }

            // Only wait and trigger gate SHOTS times; do NOT repeat first drive.
            for (int i = 1; i <= SHOTS && canRun(); i++) {

                // Feed the next ball before shots 2 through 5
                if (i > 1 && !feedNextBall(i)) break;

                boolean rpmReady = waitForRPM(MAX_RPM_WAIT_SEC, i);

                if (!canRun()) break;

                if (!shootBall(i, rpmReady)) break;
            }

            // Stop shooter before the repositioning portion.
            stopShooter();
            if (gateCycles < SHOTS) {
                outcome = "AUTO TIME LIMIT: incomplete shots";
                return;
            }
            if (!canRun()) {
                outcome = "AUTO TIME LIMIT";
                return;
            }

            if (!move(0, -LEFT_FIRST_IN, REPOSITION_POWER, "Left 26")) {
                outcome = "ABORT: left 26";
                return;
            }
            if (!move(remainingForward(), 0, REPOSITION_POWER, "Forward remaining")) {
                outcome = "ABORT: forward remaining";
                return;
            }
            if (!move(0, -LEFT_LAST_IN, REPOSITION_POWER, "Left 20")) {
                outcome = "ABORT: left 20";
                return;
            }
        } finally {
            stopAllMotors();
            telemetry.addData("Result", outcome);
            telemetry.addData("Gate cycles", "%d / %d", gateCycles, SHOTS);
            telemetry.addData("Elapsed", "%.1f sec", matchTimer.seconds());
            telemetry.update();
        }
    }

    private boolean feedNextBall(int shotNumber) {

        if (!canRun()) return false;

        gate.setPosition(GATE_CLOSED);

        telemetry.addData("Feeding ball", "%d", shotNumber);
        telemetry.update();

        ballPusher.setPower(INTAKE_FEED_POWER);

        try {
            return pauseAndControlShooter(INTAKE_FEED_MS);
        } finally {
            ballPusher.setPower(0);
        }
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ballPusher = hardwareMap.get(DcMotor.class, "ballPusher");
        gate = hardwareMap.get(Servo.class, "gate");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : driveMotors()) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPower(0);
        ballPusher.setPower(0);
        gate.setPosition(GATE_CLOSED);
    }

    private double remainingForward() {
        return baseDistanceIn + (DEFAULT_FIRST_FORWARD_IN - firstForwardIn);
    }

    private DcMotor[] driveMotors() {
        return new DcMotor[]{frontLeft, backLeft, frontRight, backRight};
    }

    /** Positive forward inches, negative strafe inches means LEFT. */
    private boolean move(double forwardIn, double strafeRightIn,
                         double speed, String label) {
        if (!canRun()) return false;
        int fTicks = -(int)Math.round(forwardIn * DRIVE_TICKS_PER_INCH);
        int sTicks = -(int)Math.round(strafeRightIn * DRIVE_TICKS_PER_INCH * STRAFE_MULTIPLIER);
        // Same mixer as TeleOp: FL=F+S; BL=F-S; FR=F-S; BR=F+S
        int[] deltas = {fTicks + sTicks, fTicks - sTicks,
                fTicks - sTicks, fTicks + sTicks};
        DcMotor[] motors = driveMotors();
        int[] targets = new int[4];
        int[] starts = new int[4];
        int largestDelta = 0;
        for (int i = 0; i < 4; i++) {
            starts[i] = motors[i].getCurrentPosition();
            targets[i] = starts[i] + deltas[i];
            largestDelta = Math.max(largestDelta, Math.abs(deltas[i]));
            motors[i].setTargetPosition(targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(Math.abs(speed));
        }

        ElapsedTime moveTimer = new ElapsedTime();
        double distance = Math.max(Math.abs(forwardIn), Math.abs(strafeRightIn));
        double timeout = Math.max(2.0, distance / (10.0 * Math.max(0.10, speed)) + 2.0);
        boolean encoderMoved = largestDelta <= POSITION_TOLERANCE_TICKS;
        boolean arrived = false;
        while (canRun() && moveTimer.seconds() < timeout) {
            updateShooter();
            int maxError = 0;
            int minTravel = Integer.MAX_VALUE;
            for (int i = 0; i < 4; i++) {
                int now = motors[i].getCurrentPosition();
                maxError = Math.max(maxError, Math.abs(targets[i] - now));
                minTravel = Math.min(minTravel, Math.abs(now - starts[i]));
            }
            if (minTravel > 15) encoderMoved = true;
            if (maxError <= POSITION_TOLERANCE_TICKS) {
                arrived = true;
                break;
            }
            if (moveTimer.seconds() > 0.7 && !encoderMoved) break;
            telemetry.addData("Step", label);
            telemetry.addData("Drive error", "%d ticks", maxError);
            telemetry.addData("Shooter RPM", "%.0f / %.0f", actualRPM, targetRPM);
            telemetry.addData("Shooter power", "%.1f%%", shooterPower * 100);
            telemetry.addData("Match elapsed", "%.1f sec", matchTimer.seconds());
            telemetry.update();
            sleep(20);
        }
        stopDrive();
        for (DcMotor motor : motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return arrived;
    }

    private void startShooter() {
        shooterEnabled = true;
        shooterFault = false;
        filterInitialized = false;
        integratedPower = 0;
        shooterPower = START_POWER;
        shooter.setPower(shooterPower);
        shooterControlTimer.reset();
        shooterSpinTimer.reset();
    }

    private void updateShooter() {
        if (!shooterEnabled || shooterFault) return;
        actualRPM = Math.abs(shooter.getVelocity()) * 60.0 / TICKS_PER_SHOOTER_REV;
        if (shooterSpinTimer.seconds() > 1.5 && actualRPM < 100.0
                && shooterPower >= 0.50) {
            shooterFault = true;
            shooterPower = 0;
            shooter.setPower(0);
            return;
        }
        double dt = shooterControlTimer.seconds();
        if (dt < SHOOTER_CONTROL_PERIOD) return;
        shooterControlTimer.reset();
        dt = Range.clip(dt, 0.01, 0.15);
        if (!filterInitialized) {
            filteredRPM = actualRPM;
            filterInitialized = true;
        } else {
            filteredRPM += RPM_FILTER_ALPHA * (actualRPM - filteredRPM);
        }
        double error = targetRPM - filteredRPM;
        double feedForward = Range.clip(START_POWER +
                (targetRPM - DEFAULT_SHOOTER_RPM) / NOMINAL_FREE_RPM, 0.0, 1.0);
        double upper = Range.clip(START_POWER + shooterSpinTimer.seconds() *
                STARTUP_RAMP_PER_SECOND, START_POWER, 1.0);
        double preliminary = feedForward + SHOOTER_KP * error + integratedPower;
        if (!(preliminary >= upper && error > 0) &&
                !(preliminary <= 0 && error < 0)) {
            integratedPower = Range.clip(integratedPower + SHOOTER_KI * error * dt,
                    -MAX_INTEGRAL, MAX_INTEGRAL);
        }
        shooterPower = Range.clip(feedForward + SHOOTER_KP * error + integratedPower,
                0.0, upper);
        shooter.setPower(shooterPower);
    }

    /** Wait for stability 350ms, no more than 3 seconds after each ball. */
    private boolean waitForRPM(double seconds, int shotNumber) {
        ElapsedTime wait = new ElapsedTime();
        ElapsedTime stable = new ElapsedTime();
        boolean wasInBand = false;
        while (canRun() && wait.seconds() < seconds) {
            updateShooter();
            boolean within = !shooterFault && filterInitialized
                    && Math.abs(filteredRPM - targetRPM) <= READY_FILTERED_BAND_RPM
                    && Math.abs(actualRPM - targetRPM) <= READY_RAW_BAND_RPM;
            if (within && !wasInBand) stable.reset();
            if (within && stable.milliseconds() >= STABLE_MS) return true;
            wasInBand = within;
            telemetry.addData("Shot", "%d / %d", shotNumber, SHOTS);
            telemetry.addData("Shooter target", "%.0f RPM", targetRPM);
            telemetry.addData("Measured RPM", "%.0f", actualRPM);
            telemetry.addData("RPM status", shooterFault ? "ENCODER FAULT" :
                    within ? "STABILIZING" : "ADJUSTING");
            telemetry.addData("RPM wait", "%.1f / 3.0 sec", wait.seconds());
            telemetry.update();
            sleep(20);
        }
        return false; // caller shoots on RPM timeout, as requested
    }

    private boolean shootBall(int shot, boolean rpmReady) {
        if (!canRun()) return false;
        telemetry.addData("Gate cycle", "%d / %d", shot, SHOTS);
        telemetry.addData("RPM", rpmReady ? "READY" : "3s TIMEOUT - SHOOT ANYWAY");
        telemetry.update();
        gate.setPosition(GATE_OPEN);
        if (!pauseAndControlShooter(GATE_OPEN_MS)) return false;
        gate.setPosition(GATE_CLOSED);
        gateCycles++;
        return pauseAndControlShooter(GATE_CLOSE_MS);
    }

    private boolean pauseAndControlShooter(long milliseconds) {
        ElapsedTime time = new ElapsedTime();
        while (canRun() && time.milliseconds() < milliseconds) {
            updateShooter();
            sleep(20);
        }
        return canRun();
    }

    private void stopShooter() {
        shooterEnabled = false;
        shooter.setPower(0);
        shooterPower = 0;
    }

    private void stopDrive() {
        for (DcMotor motor : driveMotors()) motor.setPower(0);
    }

    private void stopAllMotors() {
        stopDrive();
        stopShooter();
        ballPusher.setPower(0);
        gate.setPosition(GATE_CLOSED);
    }

    private boolean canRun() {
        return opModeIsActive() && matchTimer.seconds() < AUTO_LIMIT_SEC;
    }
}
