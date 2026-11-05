
package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Configuration selected during the Driver Station INIT phase.
 */
public final class AutoRoutineConfig {

    public enum Alliance {
        RED,
        BLUE
    }

    public enum StartSide {
        AUDIENCE,
        OPPOSITE
    }

    public enum DriveMethod {
        TIMED_TEST,
        PEDRO_TEST
    }

    public enum Routine {
        MOVE_TEST_ONLY,
        SHOOT_THEN_MOVE_TEST
    }

    // Default selections
    // Drivers can change these during INIT.

    public Alliance alliance = Alliance.BLUE;

    public StartSide side = StartSide.AUDIENCE;

    public DriveMethod driveMethod = DriveMethod.TIMED_TEST;

    public Routine routine = Routine.MOVE_TEST_ONLY;

    public String selectionName() {
        return alliance + " / " + side;
    }
}
