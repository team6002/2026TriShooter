package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;


public final class PivotConstants_TwistPivot extends PivotProperties {
    public static AngularVelocity m_MaxVelocity = DegreesPerSecond.of(1000);
    public static AngularAcceleration m_MaxAcceleration = DegreesPerSecondPerSecond.of(1000);

    public PivotConstants_TwistPivot() {
        name = "Wrist Pivot";
        primaryCanID = 11;
        primaryInverted = true;
        hasFollowerMotor = false;
        // followerCanID = 0;
        kP = 0.5;
        kI = 0.0;
        kD = 0.0;
        kS = 0.0;
        kG = 0.0;
        kV = 1.15;
        kA = 0.0;
        absoluteEncoderInverted = true;
        kArmZeroCosineOffset = Degrees.of(0);
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
