package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class PivotConstants_MainPivot extends PivotProperties {
    public static AngularVelocity m_MaxVelocity = DegreesPerSecond.of(90);
    public static AngularAcceleration m_MaxAcceleration = DegreesPerSecondPerSecond.of(90);

    public PivotConstants_MainPivot() {
        name = "Main Pivot";
        primaryCanID = 6;
        primaryInverted = false;
        hasFollowerMotor = true;
        followerCanID = 3;
        followerInverted = true;

        kP = 3.0;
        kI = 0.0;
        kD = 0.0;
        kS = 0.3;
        kG = 0.025;
        kV = 3.0;
        kA = 0.0;
        absoluteEncoderInverted = false;
        kArmZeroCosineOffset = Degrees.of(-90);
        kminAngle = Degrees.of(15);
        kmaxAngle = Degrees.of(110);
        kTolerance = Degrees.of(2);
        minOutputVoltage = -1;
        maxOutputVoltage = 1;
        positionConversionFactor = (2 * Math.PI);
        kMaxVelocity = m_MaxVelocity;
        kMaxAcceleration = m_MaxAcceleration;
    }
}
