package frc.robot.utils.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {
  public static final Translation2d BlueHubPose =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d RedHubPose =
      new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

  // Returns the current alliance, defaulting to Blue if disconnected
  public static DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
  }

  // Dynamically picks the pose based on the current alliance
  public static Translation2d getHubPose() {
    return (getAlliance() == DriverStation.Alliance.Blue)
        ? FieldConstants.BlueHubPose
        : FieldConstants.RedHubPose;
  }
}
