package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;


public final class PivotConstants_WristPivot extends PivotProperties {
    public static AngularVelocity m_MaxVelocity = DegreesPerSecond.of(800);
    public static AngularAcceleration m_MaxAcceleration = DegreesPerSecondPerSecond.of(800);

    public PivotConstants_WristPivot() {
        name = "Wrist Pivot";
        primaryCanID = 14;
        primaryInverted = false;
        hasFollowerMotor = false;
        // followerCanID = 0;
        kP = 0.6;
        kI = 0.0;
        kD = 0.0;
        kS = 0.0;
        kG = 0.15;
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
