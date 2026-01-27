package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.MapleJoystickDriveInput;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ClockDrive extends Command {
    public static final double ROTATION_AXIS_THRESHOLD = 0.5;

    private final HolonomicDriveSubsystem driveSubsystem;
    private final MapleJoystickDriveInput input;
    private final DoubleSupplier rotationXSupplier, rotationYSupplier;
    private final AtomicReference<Optional<Rotation2d>> rotationalTargetOverride;

    private Rotation2d currentDesiredFacing;

    public ClockDrive(
            HolonomicDriveSubsystem driveSubsystem,
            MapleJoystickDriveInput input,
            DoubleSupplier rotationXSupplier,
            DoubleSupplier rotationYSupplier,
            AtomicReference<Optional<Rotation2d>> rotationalTargetOverride) {

        this.driveSubsystem = driveSubsystem;
        this.input = input;
        this.rotationXSupplier = rotationXSupplier;
        this.rotationYSupplier = rotationYSupplier;
        this.rotationalTargetOverride = rotationalTargetOverride;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentDesiredFacing = driveSubsystem.getFacing();
    }

    @Override
    public void execute() {

        ChassisSpeeds pilotInputSpeed = input.getJoystickChassisSpeeds(
            driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(),
            driveSubsystem.getChassisMaxAngularVelocity()
        );

        Optional<Rotation2d> override = rotationalTargetOverride.get();

        if (override.isPresent()) {
            Rotation2d rawTarget = override.get();
            Rotation2d current = driveSubsystem.getFacing();

            // Make target continuous relative to current angle
            double continuous = current.getRadians() +
                rawTarget.minus(current).getRadians();

            Rotation2d continuousTarget = new Rotation2d(continuous);

            ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.FaceToRotationRequest(continuousTarget));

            ChassisSpeeds robotSpeeds = driveSubsystem.getMeasuredChassisSpeedsRobotRelative();

            ChassisSpeeds corrected = new ChassisSpeeds(
                0.0,
                0.0,
                robotSpeeds.omegaRadiansPerSecond
            );

            pilotInputSpeed.omegaRadiansPerSecond =
                ChassisHeadingController.getInstance().calculate(
                    corrected,
                    driveSubsystem.getPose()
                ).orElse(0);

            Logger.recordOutput("Aim/desiredFacing", continuousTarget.getDegrees());
            Logger.recordOutput("Aim/currentFacing", current.getDegrees());
            Logger.recordOutput("Aim/errorDegrees",
                current.minus(continuousTarget).getDegrees());
            Logger.recordOutput("Aim/atSetpoint", ChassisHeadingController.getInstance().atSetPoint());
        }
        else {
            Translation2d headingVector =
                new Translation2d(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble());

            if (headingVector.getNorm() > ROTATION_AXIS_THRESHOLD) {
                currentDesiredFacing = headingVector.getAngle()
                    .plus(FieldMirroringUtils.getCurrentAllianceDriverStationFacing());
            }

            ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.FaceToRotationRequest(currentDesiredFacing));
        }

        Logger.recordOutput("ClockDrive/OverridePresent", override.isPresent());
        Logger.recordOutput("ClockDrive/OverrideValue", override.orElse(null));

        driveSubsystem.runDriverStationCentricChassisSpeeds(pilotInputSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisHeadingController.getInstance()
            .setHeadingRequest(new ChassisHeadingController.NullRequest());
    }
}
