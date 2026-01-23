package frc.robot.subsystems.intake;

public class IntakeConstants {
    public static final int kIntakeCanId = 13;
    public static final int kIntakeFollowerCanId = 14;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kA = 0.0;

    public static final double kPSim = 0.0;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;
    public static final double kFFSim = 0.0;

    public static final double kSSim = 0.0;
    public static final double kVSim = 0.0;
    public static final double kGSim = 0.0;
    public static final double kASim = 0.0;

    public static final boolean kInverted = true;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kIntake = 4500;
    public static final double kOff = 0;

    public static final double kGearRatio = 2;

    public static final class ExtenderConstants{
        public static final int kIntakeExtenderCanId = 15;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;

        public static final double kV = 0.0;
        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kA = 0.0;

        public static final double kPSim = 0.0;
        public static final double kISim = 0.0;
        public static final double kDSim = 0.0;
        public static final double kFFSim = 0.0;

        public static final double kSSim = 0.0;
        public static final double kVSim = 0.0;
        public static final double kGSim = 0.0;
        public static final double kASim = 0.0;

        public static final boolean kInverted = true;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;

        public static final double kHome = Math.toRadians(0);
        public static final double kExtended = Math.toRadians(90);

        public static final double kMaxVel = Math.toRadians(360);
        public static final double kMaxAccel = Math.toRadians(360);

        public static final double kPositionTolerance = Math.toRadians(2);

        public static final double kGearRatio = 2;
    }
}
