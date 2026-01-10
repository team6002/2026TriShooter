package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public final class PivotConstants_IntakePivot extends PivotProperties {
    public static AngularVelocity m_MaxVelocity = DegreesPerSecond.of(400);
    public static AngularAcceleration m_MaxAcceleration = DegreesPerSecondPerSecond.of(400);

    public PivotConstants_IntakePivot() {
        name = "Ground Intake Pivot";
        primaryCanID = 12;
        primaryInverted = true;
        hasFollowerMotor = false;
        // followerCanID = 0;
        kP = 0.5;
        kI = 0.0;
        kD = 0.0;
        kS = 0.0;
        kG = 0.15;
        kV = 1.1;
        kA = 0.0;
        absoluteEncoderInverted = false;
        kArmZeroCosineOffset = Degrees.of(-90);
        kminAngle = Degrees.of(-20);
        kmaxAngle = Degrees.of(200);
        kTolerance = Degrees.of(2);
        minOutputVoltage = -1;
        maxOutputVoltage = 1;
        positionConversionFactor = (2 * Math.PI);
        kMaxVelocity = m_MaxVelocity;
        kMaxAcceleration = m_MaxAcceleration;
    }
}
