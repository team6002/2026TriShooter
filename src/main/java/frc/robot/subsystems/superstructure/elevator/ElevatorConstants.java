package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;


public final class ElevatorConstants extends ElevatorProperties {
    public static LinearVelocity m_MaxVelocity = MetersPerSecond.of(Meters.convertFrom(30, Inches));
    public static LinearAcceleration m_MaxAcceleration = MetersPerSecondPerSecond.of(Meters.convertFrom(30, Inches));

    // calculate how far the elevator will move with each rotation of the motor in METER
    // example:  gear reduction = 3, drum diamater = 1.625in->  (1.625 * Math.PI) / 3
    private static double _positionConversionFactor = Meters.convertFrom((1.625 * Math.PI) / 3, Inches);

    public ElevatorConstants() {
        name = "Elevator Main";
        primaryCanID = 17;
        primaryInverted = false;
        hasFollowerMotor = true;
        followerCanID = 18;
        followerInverted = true;
        kP = 10.0;
        kI = 0.0;
        kD = 0.0;
        kS = 0.25;
        kG = 0.40;
        kV = 0.0;
        kA = 0.0;
        kOffsetDistance = Meters.of(Meters.convertFrom(0, Inches));
        kTolerance = Meters.of(Meters.convertFrom(3, Inches));
        minOutputVoltage = -1;
        maxOutputVoltage = 1;
        positionConversionFactor = _positionConversionFactor;
        kMaxVelocity = m_MaxVelocity;
        kMaxAcceleration = m_MaxAcceleration;
    }
}
