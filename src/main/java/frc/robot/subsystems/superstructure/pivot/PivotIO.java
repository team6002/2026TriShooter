package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public boolean configurationFailed = false;
        public boolean primaryMotorConnected = false;
        public boolean absoluteEncoderConnected = false;
        public boolean hasFollowerMotor = false;
        public boolean followerMotorConnected = false;

        public Angle setPoint;
        public Angle currentAngle = Angle.ofRelativeUnits(0, Radians);
        public double velocityRadPerSec = 0.0;
        public double primaryMotorOutputVolts = 0.0;
        public double primaryMotorSupplyCurrentAmps = 0.0;
        public double followerMotorOutputVolts = 0.0;
        public double followerMotorSupplyCurrentAmps = 0.0;
        public double manualValue;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setIdle() {}
    ;

    public default void runAutomatic() {}

    public default void runManual(double _power) {}
    ;

    /** Run the turn motor to thmotorOutputVoltse specified rotation. */
    public default void setTargetPosition(Angle setpoint) {}

    /** Turns the motor brakes on */
    public default void setMotorOutput(Voltage voltage) {}

    /** Turns the motor brakes on */
    public default void setMotorBrake(boolean motorBrakeEnabled) {}

    public AngularVelocity getMaxAngularVelocity();

    public AngularAcceleration getMaxAngularAcceleration();
}
