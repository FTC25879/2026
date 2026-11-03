package org.firstinspires.ftc.teamcode.autonomous;

/** Pre-match selections for a single autonomous OpMode. */
public class AutoRoutineConfig {

    public enum Alliance {
        BLUE,
        RED
    }

    public enum LaunchSide {
        LEFT,
        RIGHT
    }

    public Alliance alliance = Alliance.BLUE;
    public LaunchSide launchSide = LaunchSide.LEFT;

    public String description() {
        return alliance + " / " + launchSide;
    }
}
