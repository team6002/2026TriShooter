package frc.robot.utils.constants;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class FieldConstants {
  public static final Translation2d BlueHubPose =
      new Translation2d(Units.inchesToMeters(182), Units.inchesToMeters(159));
  public static final Translation2d RedHubPose =
      new Translation2d(Units.inchesToMeters(469), Units.inchesToMeters(159));
  public static final Distance ROBOT_TO_TARGET_DISTANCE =
      Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
          // for simulation
          ? Centimeters.of(37.3)
          // for real robot (measure this on field)
          // "AdvantageKit/RealOutputs/RobotToSelectedBranchTarget" - X Axis
          : Centimeters.of(40.0);

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
