package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    public static class KickerIOInputs {
        public double kickerCurrent;
        public double kickerVoltage;
        public double kickerVelocity;
        public double kickerReference;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(KickerIOInputs inputs) {}

    public default double getCurrent() {
        return 0;
    }

    public default double getVoltage() {
        return 0;
    }

    public default double getReference() {
        return 0;
    }

    public default double getVelocity() {
        return 0;
    }

    public default void setVoltage(double voltage) {}

    public default void setReference(double velocity) {}

    public default void PID() {}

    public default void periodic() {}
}
