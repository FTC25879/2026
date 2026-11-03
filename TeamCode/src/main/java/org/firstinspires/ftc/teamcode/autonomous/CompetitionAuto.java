package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Pedro 3 autonomous: choose alliance and field side on the Driver Hub. */
@Autonomous(name = "25879 Pedro Autonomous", group = "Competition")
public class CompetitionAuto extends OpMode {

    private enum State {
        READY,
        DRIVING_TO_LAUNCH,
        LAUNCH_PAUSE,
        DRIVING_TO_PARK,
        FINISHED,
        TIMED_OUT
    }

    private final AutoRoutineConfig config = new AutoRoutineConfig();
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime pauseTimer = new ElapsedTime();

    private static final double LAUNCH_PAUSE_SECONDS = 1.0;
    private static final double AUTO_TIMEOUT_SECONDS = 28.0;

    private Follower follower;
    private AutoPaths.Route route;
    private State state = State.READY;

    @Override
    public void init() {
        // Requires your existing, correctly configured Pedro 3 Constants.java.
        follower = Constants.create(hardwareMap);
        rebuildRouteAndStartPose();
        showSelection();
    }

    @Override
    public void init_loop() {
        boolean changed = false;

        if (gamepad1.dpad_up && config.alliance != AutoRoutineConfig.Alliance.RED) {
            config.alliance = AutoRoutineConfig.Alliance.RED;
            changed = true;
        }
        if (gamepad1.dpad_down && config.alliance != AutoRoutineConfig.Alliance.BLUE) {
            config.alliance = AutoRoutineConfig.Alliance.BLUE;
            changed = true;
        }
        if (gamepad1.dpad_left && config.launchSide != AutoRoutineConfig.LaunchSide.LEFT) {
            config.launchSide = AutoRoutineConfig.LaunchSide.LEFT;
            changed = true;
        }
        if (gamepad1.dpad_right && config.launchSide != AutoRoutineConfig.LaunchSide.RIGHT) {
            config.launchSide = AutoRoutineConfig.LaunchSide.RIGHT;
            changed = true;
        }

        if (changed) {
            rebuildRouteAndStartPose();
        }
        showSelection();
    }

    private void rebuildRouteAndStartPose() {
        route = AutoPaths.build(config);
        follower.setPose(route.startPose);
    }

    private void showSelection() {
        telemetry.addLine("D-pad UP: Red    DOWN: Blue");
        telemetry.addLine("D-pad LEFT: Left  RIGHT: Right");
        telemetry.addData("Selected", config.description());
        telemetry.addData("Start X", "%.1f", route.startPose.x());
        telemetry.addData("Start Y", "%.1f", route.startPose.y());
        telemetry.addData("Start heading", "%.1f deg",
                Math.toDegrees(route.startPose.heading()));
        telemetry.addLine("Position robot at this pose before START.");
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        follower.setPose(route.startPose);
        follower.follow(route.toLaunch);
        state = State.DRIVING_TO_LAUNCH;
    }

    @Override
    public void loop() {
        follower.update();

        if (autoTimer.seconds() >= AUTO_TIMEOUT_SECONDS
                && state != State.FINISHED
                && state != State.TIMED_OUT) {
            follower.stop();
            state = State.TIMED_OUT;
        }

        switch (state) {
            case DRIVING_TO_LAUNCH:
                if (!follower.following()) {
                    // TODO: Place your shooter / gate activation here.
                    // At present, this code simply pauses for one second.
                    pauseTimer.reset();
                    state = State.LAUNCH_PAUSE;
                }
                break;

            case LAUNCH_PAUSE:
                if (pauseTimer.seconds() >= LAUNCH_PAUSE_SECONDS) {
                    // TODO: Ensure shooter / gate is turned off here.
                    follower.follow(route.toPark);
                    state = State.DRIVING_TO_PARK;
                }
                break;

            case DRIVING_TO_PARK:
                if (!follower.following()) {
                    follower.stop();
                    state = State.FINISHED;
                }
                break;

            case READY:
            case FINISHED:
            case TIMED_OUT:
            default:
                break;
        }

        telemetry.addData("Selection", config.description());
        telemetry.addData("State", state);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading", "%.1f deg",
                Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
    }
}
