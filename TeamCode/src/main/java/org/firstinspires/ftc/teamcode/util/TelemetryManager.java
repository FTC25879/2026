package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public final class TelemetryManager {
    private TelemetryManager() {}
    public static void addRobot(Telemetry t, Robot robot) {
        t.addData("Battery", "%.2f V", robot.voltage());
        t.addData("Heading", "%.1f deg", robot.headingDegrees());
        t.addData("Shooter", "%s target %.0f / actual %.0f RPM",
            robot.shooter.isEnabled() ? "ON" : "OFF",
            robot.shooter.getTargetRPM(), robot.shooter.getActualRPM());
        t.addData("Shooter current", "%.2f A", robot.shooter.getCurrentAmps());
        t.addData("Shooter ready", robot.shooter.isReady());
        t.addData("Shooter fault", robot.shooter.hasFault() ? robot.shooter.getFault() : "none");
        t.addData("Gate", robot.gate.isOpen() ? "OPEN" : "CLOSED");
        t.addData("Shots", "%d (%s)", robot.shots.getCompletedShots(), robot.shots.getState());
        t.addData("Stick", robot.stick.isExtended() ? "EXTENDED" : "RETRACTED");
        t.addData("Intake", "%.2f", robot.intake.getPower());
    }
}
