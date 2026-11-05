package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;

/**
 * INTEGRATION TEST autonomous; not a validated BIOBUZZ scoring/parking route.
 * Defaults to a short, slow timed forward drive. Shooter and Pedro require explicit
 * RobotConfig enable flags and Driver Station INIT selection.
 */
@Autonomous(name = "25879 Competition Auto (TEST)", group = "Competition")
public final class CompetitionAuto extends LinearOpMode {
    private enum Step { WAIT_SHOOTER, FEED, BEGIN_MOVE, MOVING, DONE }
    private final ElapsedTime stage = new ElapsedTime();
    private final ElapsedTime total = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        AutoRoutineConfig options = new AutoRoutineConfig();
        boolean priorA = false, priorY = false;
        Follower follower = null;
        boolean useFieldPoses = false;
        Step step = Step.BEGIN_MOVE;
        String status = "Ready for TEST";

        try {
            while (!isStarted() && !isStopRequested()) {
                // These selections are recorded but do NOT yet transform test coordinates.
                if (gamepad1.x) options.alliance = AutoRoutineConfig.Alliance.BLUE;
                if (gamepad1.b) options.alliance = AutoRoutineConfig.Alliance.RED;
                if (gamepad1.dpad_left) options.side = AutoRoutineConfig.StartSide.AUDIENCE;
                if (gamepad1.dpad_right) options.side = AutoRoutineConfig.StartSide.OPPOSITE;

                boolean a = gamepad1.a, y = gamepad1.y;
                if (a && !priorA && RobotConfig.ENABLE_AUTO_SHOOTING) {
                    options.routine = options.routine == AutoRoutineConfig.Routine.MOVE_TEST_ONLY
                        ? AutoRoutineConfig.Routine.SHOOT_THEN_MOVE_TEST
                        : AutoRoutineConfig.Routine.MOVE_TEST_ONLY;
                }
                if (y && !priorY && RobotConfig.ENABLE_PEDRO_AUTONOMOUS
                    && RobotConfig.LOCALIZATION_CALIBRATED) {
                    options.driveMethod = options.driveMethod == AutoRoutineConfig.DriveMethod.TIMED_TEST
                        ? AutoRoutineConfig.DriveMethod.PEDRO_TEST
                        : AutoRoutineConfig.DriveMethod.TIMED_TEST;
                }
                priorA = a;
                priorY = y;

                telemetry.addLine("TEAM 25879 AUTONOMOUS - TEST ROUTE ONLY");
                telemetry.addData("Alliance [X Blue / B Red]", options.alliance);
                telemetry.addData("Side [D-pad L/R]", options.side);
                telemetry.addData("Routine [A toggle]", options.routine);
                telemetry.addData("Drive [Y toggle]", options.driveMethod);
                telemetry.addData("Pedro unlocked", RobotConfig.ENABLE_PEDRO_AUTONOMOUS && RobotConfig.LOCALIZATION_CALIBRATED);
                telemetry.addData("Auto shooting unlocked", RobotConfig.ENABLE_AUTO_SHOOTING);
                telemetry.addData("Start pose calibration", RobotConfig.AUTO_FIELD_POSES_VERIFIED ? "VERIFIED" : "UNVERIFIED - robot-relative TEST");
                telemetry.addLine("X=BLUE B=RED | D-pad LEFT=AUDIENCE RIGHT=OPPOSITE");
                telemetry.addLine("START selects the displayed settings for THIS autonomous only.");
                telemetry.addLine("Test move is NOT a BIOBUZZ parking/scoring route.");
                telemetry.update();
                sleep(50);
            }
            if (isStopRequested()) return;

            total.reset();
            stage.reset();
            robot.resetHeading();
            if (options.driveMethod == AutoRoutineConfig.DriveMethod.PEDRO_TEST) {
                follower = Constants.create(hardwareMap, robot.drive);
                useFieldPoses = RobotConfig.AUTO_FIELD_POSES_VERIFIED;
                follower.setPose(useFieldPoses ? AutoPaths.selectedStart(options) : AutoPaths.testStart());
            }

            if (options.routine == AutoRoutineConfig.Routine.SHOOT_THEN_MOVE_TEST) {
                robot.shooter.start();
                step = Step.WAIT_SHOOTER;
                status = "Waiting for shooter";
            }

            while (opModeIsActive()) {
                robot.update(); // Includes RPM safety and nonblocking gate timing.
                if (total.seconds() > 26.0) {
                    status = "Auto watchdog timeout";
                    step = Step.DONE;
                }
                if (robot.shooter.hasFault() && (step == Step.WAIT_SHOOTER || step == Step.FEED)) {
                    robot.shots.cancel();
                    robot.shooter.stop();
                    step = Step.BEGIN_MOVE;
                    stage.reset();
                    status = "Shooter fault - skip shots";
                }

                switch (step) {
                    case WAIT_SHOOTER:
                        if (robot.shooter.isReady()) {
                            if (robot.shots.requestOne()) {
                                step = Step.FEED;
                                stage.reset();
                                status = "Feeding balls";
                            }
                        } else if (stage.seconds() > RobotConfig.SHOT_READY_TIMEOUT_SECONDS) {
                            robot.shooter.stop();
                            step = Step.BEGIN_MOVE;
                            stage.reset();
                            status = "RPM timeout - skip shots";
                        }
                        break;

                    case FEED:
                        if (robot.shots.didTimeout() || stage.seconds() > 12.0) {
                            robot.shots.cancel();
                            robot.shooter.stop();
                            step = Step.BEGIN_MOVE;
                            stage.reset();
                            status = "Feed timeout - skip remainder";
                        } else if (!robot.shots.isBusy()) {
                            if (robot.shots.getCompletedShots() >= RobotConfig.AUTO_MAX_SHOTS) {
                                robot.shooter.stop();
                                step = Step.BEGIN_MOVE;
                                stage.reset();
                                status = "Shots completed";
                            } else if (robot.shooter.isReady()) {
                                robot.shots.requestOne();
                            }
                        }
                        break;

                    case BEGIN_MOVE:
                        robot.shots.cancel();
                        robot.intake.stop();
                        robot.shooter.stop();
                        if (follower != null) {
                            follower.follow(useFieldPoses
                                ? AutoPaths.selectedFieldTest(options)
                                : AutoPaths.testForward());
                        }
                        stage.reset();
                        step = Step.MOVING;
                        status = follower == null ? "TIMED forward test" : "PEDRO forward test";
                        break;

                    case MOVING:
                        if (follower == null) {
                            if (stage.seconds() < RobotConfig.AUTO_TEST_DRIVE_SECONDS) {
                                robot.drive.drive(new DrivePowers(RobotConfig.AUTO_TEST_DRIVE_POWER, 0, 0), false);
                            } else {
                                robot.drive.stop();
                                step = Step.DONE;
                                status = "Timed test ended";
                            }
                        } else {
                            follower.update();
                            if (!follower.following() || stage.seconds() > RobotConfig.AUTO_PATH_TIMEOUT_SECONDS) {
                                if (stage.seconds() > RobotConfig.AUTO_PATH_TIMEOUT_SECONDS) {
                                    status = "Pedro path timeout";
                                } else {
                                    status = "Pedro test ended";
                                }
                                follower.stop();
                                robot.drive.stop();
                                step = Step.DONE;
                            }
                        }
                        break;

                    case DONE:
                        if (follower != null) follower.stop();
                        robot.drive.stop();
                        robot.intake.stop();
                        robot.shooter.stop();
                        robot.shots.cancel();
                        break;
                }
                telemetry.addData("Auto stage", step);
                telemetry.addData("Status", status);
                telemetry.addData("Elapsed", "%.1f sec", total.seconds());
                telemetry.addData("Alliance/Side", options.selectionName());
                telemetry.addData("Field poses", useFieldPoses ? "VERIFIED (test path)" : "ROBOT-RELATIVE TEST ONLY");
                if (follower != null) telemetry.addData("Pedro pose", follower.pose());
                TelemetryManager.addRobot(telemetry, robot);
                telemetry.update();
                idle();
            }
        } finally {
            if (follower != null) follower.stop();
            robot.stopAll();
        }
    }
}
