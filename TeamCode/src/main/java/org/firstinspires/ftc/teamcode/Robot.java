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
import org.firstinspires.ftc.teamcode.subsystems.OriginalControlMechanisms;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.subsystems.WorkingMecanumDrive;

/**
 * Scaffolding facade. Two deliberately SEPARATE hardware ownership modes:
 *  - Robot(hardwareMap): unchanged Pedro Auto + advanced mechanisms.
 *  - Robot(hardwareMap, true): tested manual mecanum and original A/X/Y devices.
 * The advanced and manual mechanism fields are mutually exclusive so that
 * they cannot issue competing power/servo commands.
 */
public final class Robot {
    public final Mecanum drive; // Pedro Auto only
    public final WorkingMecanumDrive manualDrive; // manual TeleOp only
    public final OriginalControlMechanisms manualMechanisms; // manual TeleOp only

    public final Intake intake; // original advanced/Auto only
    public final Shooter shooter; // original advanced/Auto only
    public final Gate gate; // original advanced/Auto only
    public final FlowerStick stick; // original advanced/Auto only
    public final ShotSequencer shots; // original advanced/Auto only

    private final HardwareMap map;
    private final IMU imu;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public Robot(HardwareMap hardwareMap, boolean useOriginalManualControls) {
        map = hardwareMap;

        if (useOriginalManualControls) {
            // Do not initialize Pedro, the IMU, or the advanced shooter/gate/stick
            // classes; each might overwrite a known-working motor/servo command.
            drive = null;
            imu = null;
            manualDrive = new WorkingMecanumDrive(map);
            manualMechanisms = new OriginalControlMechanisms(map);
            intake = null;
            shooter = null;
            gate = null;
            stick = null;
            shots = null;
        } else {
            manualDrive = null;
            manualMechanisms = null;
            drive = new Mecanum(map, Constants.drivetrainConfig);
            imu = map.get(IMU.class, RobotConfig.IMU);
            imu.initialize(new IMU.Parameters(RobotConfig.HUB_ORIENTATION));
            intake = new Intake(map);
            shooter = new Shooter(map);
            gate = new Gate(map);
            stick = new FlowerStick(map);
            shots = new ShotSequencer(gate, shooter);
        }
    }

    public boolean hasIMU() { return imu != null; }
    public double headingRadians() {
        return imu == null ? 0.0
            : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double headingDegrees() {
        return imu == null ? 0.0
            : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void resetHeading() {
        if (imu != null) imu.resetYaw();
    }

    public double voltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0 && v < result) result = v;
        }
        return Double.isInfinite(result) ? -1 : result;
    }

    /** Advanced/Auto update only. Original manual hardware is commanded in TeleOp. */
    public void update() {
        if (shooter != null && shots != null) {
            shooter.update(voltage());
            shots.update();
        }
    }

    public void stopAll() {
        if (manualMechanisms != null) manualMechanisms.stop();
        if (manualDrive != null) manualDrive.stop();
        if (shots != null) shots.cancel();
        if (intake != null) intake.stop();
        if (shooter != null) shooter.stop();
        if (gate != null) gate.close();
        if (stick != null) stick.retract();
        if (drive != null) drive.stop();
    }
}
