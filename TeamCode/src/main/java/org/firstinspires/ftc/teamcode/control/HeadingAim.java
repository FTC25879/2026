package org.firstinspires.ftc.teamcode.control;

/** Heading hold only, not a field target tracker. Angles in radians. */
public final class HeadingAim {
    private double target;
    public static double KP = 1.0; // Turn power per radian: tune on robot
    public static double MAX_TURN = 0.23;
    public static double DEADBAND_RADIANS = Math.toRadians(2.0);
    public static double TURN_SIGN = 1.0; // Flip ONLY after proving correction goes wrong way

    public void capture(double currentHeading) { target = currentHeading; }
    public double getError(double currentHeading) {
        double error = target - currentHeading;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }
    public double getTurn(double currentHeading) {
        double error = getError(currentHeading);
        if (Math.abs(error) <= DEADBAND_RADIANS) return 0;
        return TURN_SIGN * Math.max(-MAX_TURN, Math.min(MAX_TURN, KP * error));
    }
}
