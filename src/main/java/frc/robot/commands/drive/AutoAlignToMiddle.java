package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignToMiddle extends SequentialCommandGroup {
    public AutoAlignToMiddle(Drive drive) {
        if (FieldConstants.getAlliance() == Alliance.Red) {
            addCommands(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new GoToPose(new Pose2d(13.3, 0.9, new Rotation2d()), drive, 0.05)
                        ,new GoToPose(new Pose2d(9.5, 0.9, new Rotation2d()), drive, 0.05)
                    )
                    ,new SequentialCommandGroup(
                        new GoToPose(new Pose2d(13.3, 7.4, new Rotation2d()), drive, 0.05)
                        ,new GoToPose(new Pose2d(9.5, 7.4, new Rotation2d()), drive, 0.05)
                    ) 
                    ,()->drive.getPose().getY() < 4
                )
            );
        } else {
            addCommands(
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new GoToPose(new Pose2d(3.2, 1.1, new Rotation2d()), drive, 0.05)
                        ,new GoToPose(new Pose2d(6, 0.9, new Rotation2d()), drive, 0.05)
                    )
                    ,new SequentialCommandGroup(
                        new GoToPose(new Pose2d(3.2, 7.2, new Rotation2d()), drive, 0.05)
                        ,new GoToPose(new Pose2d(6, 7.2, new Rotation2d()), drive, 0.05)
                    ) 
                    ,()->drive.getPose().getY() < 4
                )
            );
        }
    }
}