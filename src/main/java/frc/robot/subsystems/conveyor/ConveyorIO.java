package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
    @AutoLog
    public static class ConveyorIOInputs {
        public double conveyorCurrent;
        public double conveyorVoltage;
        public double conveyorVelocity;
        public double conveyorReference;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ConveyorIOInputs inputs) {}

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
