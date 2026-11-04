package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

/** Nonblocking, one-ball gate pulse with measured-RPM wait and timeout. */
public final class ShotSequencer {
    private enum State { IDLE, WAIT_RPM, GATE_OPEN, SETTLE }
    private State state = State.IDLE;
    private final Gate gate;
    private final Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private int completedShots;
    private boolean timedOut;

    public ShotSequencer(Gate gate, Shooter shooter) {
        this.gate = gate;
        this.shooter = shooter;
    }
    public boolean requestOne() {
        if (state != State.IDLE || shooter.hasFault() || !shooter.isEnabled()) return false;
        state = State.WAIT_RPM;
        timedOut = false;
        timer.reset();
        return true;
    }
    public void update() {
        switch (state) {
            case WAIT_RPM:
                if (!shooter.isEnabled() || shooter.hasFault()) { cancel(); break; }
                if (shooter.isReady()) {
                    gate.open();
                    timer.reset();
                    state = State.GATE_OPEN;
                } else if (timer.seconds() > RobotConfig.SHOT_READY_TIMEOUT_SECONDS) {
                    timedOut = true;
                    cancel();
                }
                break;
            case GATE_OPEN:
                if (timer.seconds() >= RobotConfig.GATE_OPEN_SECONDS) {
                    gate.close();
                    timer.reset();
                    state = State.SETTLE;
                }
                break;
            case SETTLE:
                if (timer.seconds() >= RobotConfig.GATE_SETTLE_SECONDS) {
                    completedShots++;
                    state = State.IDLE;
                }
                break;
            default:
                break;
        }
    }
    public void cancel() { gate.close(); state = State.IDLE; }
    public boolean isBusy() { return state != State.IDLE; }
    public boolean didTimeout() { return timedOut; }
    public int getCompletedShots() { return completedShots; }
    public String getState() { return state.name(); }
    public void reset() { cancel(); completedShots = 0; timedOut = false; }
}
