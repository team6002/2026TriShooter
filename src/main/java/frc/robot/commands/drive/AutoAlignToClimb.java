package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.constants.FieldConstants;

public class AutoAlignToClimb extends SequentialCommandGroup {
  public AutoAlignToClimb(Drive drive) {
    if (FieldConstants.getAlliance() == Alliance.Red) {
      addCommands(
          new ConditionalCommand(
              new SequentialCommandGroup(
                  new GoToPose(new Pose2d(15.3, 5.6, new Rotation2d()), drive, 0.3),
                  new GoToPose(new Pose2d(15.3, 5.3, new Rotation2d()), drive, 0.1)),
              new SequentialCommandGroup(
                  new GoToPose(new Pose2d(15.3, 2.8, new Rotation2d(Math.PI)), drive, 0.3),
                  new GoToPose(new Pose2d(15.3, 3.3, new Rotation2d(Math.PI)), drive, 0.1)),
              () -> drive.getPose().getY() > 4));
    } else {
      new Rotation2d();
      new Rotation2d();
      addCommands(
          new ConditionalCommand(
              new SequentialCommandGroup(
                  new GoToPose(new Pose2d(1.3, 2.6, Rotation2d.fromDegrees(180)), drive, 0.3),
                  new GoToPose(new Pose2d(1.3, 2.8, Rotation2d.fromDegrees(180)), drive, 0.1)),
              new SequentialCommandGroup(
                  new GoToPose(new Pose2d(1.3, 5.1, new Rotation2d()), drive, 0.3),
                  new GoToPose(new Pose2d(1.3, 4.9, new Rotation2d()), drive, 0.1)),
              () -> drive.getPose().getY() < 4));
    }
  }
}
