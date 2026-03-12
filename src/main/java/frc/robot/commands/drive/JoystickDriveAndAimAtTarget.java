package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.CustomPIDs.ChassisHeadingController;
import frc.robot.utils.CustomPIDs.MapleJoystickDriveInput;
import frc.robot.utils.CustomPIDs.MapleShooterOptimization;
import java.util.function.Supplier;

public class JoystickDriveAndAimAtTarget {
  public static Command driveAndAimAtTarget(
      MapleJoystickDriveInput input,
      HolonomicDriveSubsystem driveSubsystem,
      Supplier<Translation2d> targetPositionSupplier,
      MapleShooterOptimization shooterOptimization,
      double pilotInputMultiplier,
      boolean finishWhenComplete) {
    return new FunctionalCommand(
        () ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(
                    new ChassisHeadingController.FaceToTargetRequest(
                        targetPositionSupplier, shooterOptimization)),
        () -> {
          if (ChassisHeadingController.getInstance().getAbsoluteHeadingErrorDegrees() <= 1.0) {
            driveSubsystem.stopWithX();
          } else {
            execute(driveSubsystem, input, pilotInputMultiplier);
          }
        },
        (interrupted) ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.NullRequest()),
        () -> finishWhenComplete && ChassisHeadingController.getInstance().atSetPoint(),
        driveSubsystem);
  }

  public static Command driveAndAimAtDirection(
      MapleJoystickDriveInput input,
      HolonomicDriveSubsystem driveSubsystem,
      Supplier<Rotation2d> rotationTarget,
      double pilotInputMultiplier,
      boolean finishWhenComplete) {
    return new FunctionalCommand(
        () ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(
                    new ChassisHeadingController.FaceToRotationRequest(rotationTarget.get())),
        () -> {
          if (ChassisHeadingController.getInstance().getAbsoluteHeadingErrorDegrees() <= 1.0) {
            driveSubsystem.stopWithX();
          } else {
            execute(driveSubsystem, input, pilotInputMultiplier);
          }
        },
        (interrupted) ->
            ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.NullRequest()),
        () -> finishWhenComplete && ChassisHeadingController.getInstance().atSetPoint(),
        driveSubsystem);
  }

  public static void execute(
      HolonomicDriveSubsystem driveSubsystem,
      MapleJoystickDriveInput input,
      double pilotInputMultiplier) {
    driveSubsystem.runDriverStationCentricChassisSpeeds(
        input.getJoystickChassisSpeeds(
            driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() * pilotInputMultiplier, 0),
        true);
  }
}
