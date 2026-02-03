package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterCurrent;
        public double shooterVoltage;
        public double shooterVelocity;
        public double shooterReference;
        public double shooterPos;

        public double leftShooterCurrent;
        public double leftShooterVoltage;
        public double leftShooterVelocity;

        public double rightShooterCurrent;
        public double rightShooterVoltage;
        public double rightShooterVelocity;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

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
