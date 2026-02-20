package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double climbCurrent;
        public double climbVoltage;
        public double climbVelocity;
        public double climbReference;
        public double climbTemp;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimbIOInputs inputs) {}

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

    public default void periodic() {}
}
