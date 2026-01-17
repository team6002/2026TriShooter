package frc.robot.commands;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class goToPose extends SequentialCommandGroup {
    public goToPose(Pose2d targetPose, SwerveDriveSimulation driveSimulation, Drive drive, double tolerance) {
        var robotPose = driveSimulation.getSimulatedDriveTrainPose();

            Rotation2d targetAngle = targetPose.getRotation();

            addCommands(
                // Step 1: Rotate to face the target
                new InstantCommand(() -> {
                    Rotation2d currentRotation = robotPose.getRotation();
                    Rotation2d relativeRotation = targetAngle.minus(currentRotation);
                    driveSimulation.rotateAboutCenter(relativeRotation.getRadians()+Math.PI);
                }),

                // Step 2: Drive toward target using joystick-style drive
                DriveCommands.joystickDrive(
                    drive,
                    () -> {
                        double dx = targetPose.getX() - driveSimulation.getSimulatedDriveTrainPose().getX();
                        double dy = targetPose.getY() - driveSimulation.getSimulatedDriveTrainPose().getY();
                        double mag = Math.hypot(dx, dy);
                        return dx / (mag == 0 ? 1 : mag);
                    },
                    () -> {
                        double dx = targetPose.getX() - driveSimulation.getSimulatedDriveTrainPose().getX();
                        double dy = targetPose.getY() - driveSimulation.getSimulatedDriveTrainPose().getY();
                        double mag = Math.hypot(dx, dy);
                        return dy / (mag == 0 ? 1 : mag);
                    },
                    () -> 0
                ).until(() -> {
                    double dx = targetPose.getX() - driveSimulation.getSimulatedDriveTrainPose().getX();
                    double dy = targetPose.getY() - driveSimulation.getSimulatedDriveTrainPose().getY();
                    return Math.hypot(dx, dy) < tolerance; // stop within 25 cm
                }),

                // Step 3: Stop robot
                new InstantCommand(() ->
                    drive.runRobotCentricChassisSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))
                )
            );
    }

    public goToPose(Pose2d targetPose, Drive drive, double tolerance) {
        var robotPose = drive.getPose();

            Rotation2d targetAngle = targetPose.getRotation();

            addCommands(
                // Step 1: Rotate to face the target
                new InstantCommand(() -> {
                    drive.runRobotCentricChassisSpeeds(new ChassisSpeeds(0, 0, 0.1));
                }).until(()->{
                    Rotation2d currentRotation = robotPose.getRotation();
                    Rotation2d relativeRotation = targetAngle.minus(currentRotation);
                    return relativeRotation.getRadians()+Math.PI == drive.getPose().getRotation().getRadians();
                }),

                // Step 2: Drive toward target using joystick-style drive
                DriveCommands.joystickDrive(
                    drive,
                    () -> {
                        double dx = targetPose.getX() - drive.getPose().getX();
                        double dy = targetPose.getY() - drive.getPose().getY();
                        double mag = Math.hypot(dx, dy);
                        return dx / (mag == 0 ? 1 : mag);
                    },
                    () -> {
                        double dx = targetPose.getX() - drive.getPose().getX();
                        double dy = targetPose.getY() - drive.getPose().getY();
                        double mag = Math.hypot(dx, dy);
                        return dy / (mag == 0 ? 1 : mag);
                    },
                    () -> 0
                ).until(() -> {
                    double dx = targetPose.getX() - drive.getPose().getX();
                    double dy = targetPose.getY() - drive.getPose().getY();
                    return Math.hypot(dx, dy) < tolerance; // stop within 25 cm
                }),

                // Step 3: Stop robot
                new InstantCommand(() ->
                    drive.runRobotCentricChassisSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))
                )
            );
    }
}
