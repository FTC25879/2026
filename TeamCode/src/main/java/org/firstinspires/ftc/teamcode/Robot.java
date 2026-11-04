package org.firstinspires.ftc.teamcode;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerStick;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;

/** Robot's shared subsystems. Both autonomous and TeleOp use the same methods. */
public final class Robot {
    public final Mecanum drive;
    public final Intake intake;
    public final Shooter shooter;
    public final Gate gate;
    public final FlowerStick stick;
    public final ShotSequencer shots;
    private final HardwareMap map;
    private final IMU imu;

    public Robot(HardwareMap hardwareMap) {
        map = hardwareMap;
        drive = new Mecanum(map, Constants.drivetrainConfig);
        imu = map.get(IMU.class, RobotConfig.IMU);
        imu.initialize(new IMU.Parameters(RobotConfig.HUB_ORIENTATION));
        intake = new Intake(map);
        shooter = new Shooter(map);
        gate = new Gate(map);
        stick = new FlowerStick(map);
        shots = new ShotSequencer(gate, shooter);
    }

    public double headingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double headingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void resetHeading() { imu.resetYaw(); }

    /** Returns first valid battery voltage; -1 means unavailable. */
    public double voltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0 && v < result) result = v;
        }
        return Double.isInfinite(result) ? -1 : result;
    }

    /** Call EVERY loop, including when the drivetrain is controlled by Pedro. */
    public void update() {
        shooter.update(voltage());
        shots.update();
    }

    public void stopAll() {
        shots.cancel();
        intake.stop();
        shooter.stop();
        gate.close();
        stick.retract();
        drive.stop();
    }
}
