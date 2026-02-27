package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.constants.FieldConstants;

public class GoToPose extends SequentialCommandGroup {
    public GoToPose(Pose2d targetPose, Drive drive, double tolerance) {
        Rotation2d targetAngle = targetPose.getRotation();

        double allianceFactor = FieldConstants.getAlliance() == Alliance.Red ? -1 : 1;

        addCommands(
                // Step 1: Rotate to face the target
                drive.alignToAngle(targetAngle, Math.toRadians(5)),

                // Step 2: Drive toward target using joystick-style drive
                frc.robot.commands.drive.DriveCommands.joystickDrive(
                                drive,
                                () -> {
                                    double dx =
                                            targetPose.getX() - drive.getPose().getX();
                                    double dy =
                                            targetPose.getY() - drive.getPose().getY();
                                    double mag = Math.hypot(dx, dy);
                                    return allianceFactor * dx / (mag == 0 ? 1 : mag);
                                },
                                () -> {
                                    double dx =
                                            targetPose.getX() - drive.getPose().getX();
                                    double dy =
                                            targetPose.getY() - drive.getPose().getY();
                                    double mag = Math.hypot(dx, dy);
                                    return allianceFactor * dy / (mag == 0 ? 1 : mag);
                                },
                                () -> 0)
                        .until(() -> {
                            double dx = targetPose.getX() - drive.getPose().getX();
                            double dy = targetPose.getY() - drive.getPose().getY();
                            return Math.hypot(dx, dy) < tolerance; // stop within 25 cm
                        }),

                // Step 3: Stop robot
                new InstantCommand(() -> drive.runVelocity(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))));
    }
}
