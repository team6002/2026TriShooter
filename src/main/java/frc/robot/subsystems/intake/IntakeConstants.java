package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int kIntakeCanId = 12;
  public static final int kIntakeFollowerCanId = 1;

  public static final double kP = 0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;

  public static final double kV = 0.0;
  public static final double kS = 0.0;
  public static final double kG = 0.0;
  public static final double kA = 0;

  public static final double kPSim = 0.0;
  public static final double kISim = 0.0;
  public static final double kDSim = 0.0;

  public static final double kSSim = 0.0;
  public static final double kVSim = 0.0;
  public static final double kGSim = 0.0;
  public static final double kASim = 0.0;

  public static final boolean kInverted = false;
  public static final double kMinOutput = -1;
  public static final double kMaxOutput = 1;

  public static final double kOn = 12;
  public static final double kOff = 0;

  public static final double kGearRatio = 1;

  public static final class ExtenderConstants {
    public static final int kIntakeExtenderCanId = 7;

    public static final double kP = 5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.33;
    public static final double kA = 0.0;

    public static final double kPSim = 0.0;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;
    public static final double kFFSim = 0.0;

    public static final double kSSim = 0.0;
    public static final double kVSim = 0.0;
    public static final double kGSim = 0.0;
    public static final double kASim = 0.0;

    public static final boolean kInverted = false;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kHome = Math.toRadians(42.5);
    public static final double kStow = Math.toRadians(160);
    public static final double kExtended = Math.toRadians(260);

    public static final double kMaxVel = Math.toRadians(12000); // 72000
    public static final double kMaxAccel = Math.toRadians(36000); // 36000

    public static final double kPositionTolerance = Math.toRadians(5);

    public static final double kGearRatio = 81;
    public static final double kAbsoluteGearRatio = 3;
  }
}
