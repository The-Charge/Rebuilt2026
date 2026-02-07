package frc.robot.utils;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public enum TeleopPhase {
    TRANSITION_SHIFT,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME;

    public static final Time transitionShiftEndTime = Seconds.of(10);
    public static final Time shift1EndTime = Seconds.of(10 + 25);
    public static final Time shift2EndTime = Seconds.of(10 + 2 * 25);
    public static final Time shift3EndTime = Seconds.of(10 + 3 * 25);
    public static final Time shift4EndTime = Seconds.of(10 + 4 * 25);
    public static final Time endgameEndTime = Seconds.of(10 + 4 * 25 + 30);

    /**
     * Returns the current shift given the time since teleop has started
     * @return
     */
    public static TeleopPhase fromTeleopTimer(Time teleopTime) {
        if (teleopTime == null) return TRANSITION_SHIFT;

        double secs = teleopTime.abs(Seconds);
        if (secs < transitionShiftEndTime.abs(Seconds)) return TRANSITION_SHIFT;
        if (secs < shift1EndTime.abs(Seconds)) return SHIFT1;
        if (secs < shift2EndTime.abs(Seconds)) return SHIFT2;
        if (secs < shift3EndTime.abs(Seconds)) return SHIFT3;
        if (secs < shift4EndTime.abs(Seconds)) return SHIFT4;
        return ENDGAME;
    }

    public static Time getTimeLeftInPhase(Time teleopTime) {
        if (teleopTime == null) return Seconds.of(0);

        double secs = teleopTime.abs(Seconds);
        TeleopPhase phase = TeleopPhase.fromTeleopTimer(teleopTime);

        switch (phase) {
            case TRANSITION_SHIFT: {
                return Seconds.of(transitionShiftEndTime.abs(Seconds) - secs);
            }
            case SHIFT1: {
                return Seconds.of(shift1EndTime.abs(Seconds) - secs);
            }
            case SHIFT2: {
                return Seconds.of(shift2EndTime.abs(Seconds) - secs);
            }
            case SHIFT3: {
                return Seconds.of(shift3EndTime.abs(Seconds) - secs);
            }
            case SHIFT4: {
                return Seconds.of(shift4EndTime.abs(Seconds) - secs);
            }
            case ENDGAME: {
                return Seconds.of(endgameEndTime.abs(Seconds) - secs);
            }
            default: {
                return Seconds.of(0);
            }
        }
    }
}
