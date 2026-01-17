package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeCurrent;
        public double intakeVoltage;
        public double intakeVelocity;
        public double intakeReference;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

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
