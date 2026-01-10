package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean configurationFailed = false;
        public boolean primaryMotorConnected = false;
        public boolean absoluteEncoderConnected = false;
        public boolean hasFollowerMotor = false;
        public boolean followerMotorConnected = false;

        public Distance setPoint;
        public Distance currentHeight = Distance.ofRelativeUnits(0, Inches);
        public LinearVelocity velocity = InchesPerSecond.of(0);
        public double primaryMotorOutputVolts = 0.0;
        public double primaryMotorSupplyCurrentAmps = 0.0;
        public double followerMotorOutputVolts = 0.0;
        public double followerMotorSupplyCurrentAmps = 0.0;
        public double manualValue;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runAutomatic() {}

    public default void runManual(double _power) {}
    ;

    /** Run the turn motor to thmotorOutputVoltse specified rotation. */
    public default void setTargetPosition(Distance setpoint) {}

    /** Turns the motor brakes on */
    public default void setMotorOutput(Voltage voltage) {}

    /** Turns the motor brakes on */
    public default void setMotorBrake(boolean motorBrakeEnabled) {}
}
