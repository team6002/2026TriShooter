package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.utils.CustomPIDs.ChassisHeadingController;
import frc.robot.utils.CustomPIDs.MapleJoystickDriveInput;
import frc.robot.utils.constants.FieldConstants;

public class CMD_Shoot extends ParallelCommandGroup {
    private final Debouncer atSetpointDebouncer = new Debouncer(.5);
  public CMD_Shoot(
      Drive drive,
      MapleJoystickDriveInput driveSupplier,
      Conveyor conveyor,
      Hood hood,
      Intake intake,
      Kicker kicker,
      Shooter shooter) {

    addCommands(
        JoystickDriveAndAimAtTarget.driveAndAimAtTarget(
            driveSupplier,
            drive,
            FieldConstants::getHubPose,
            ShooterConstants.kShooterOptimization,
            0.5,
            false),

        new RunCommand(
            () -> {
              shooter.setReference(Math.toRadians(20000));
              hood.setReference(0.4);

              if (shooter.isReady() 
                  && hood.atReference() 
                  && atSetpointDebouncer.calculate(ChassisHeadingController.getInstance().atSetPoint())) {
                conveyor.setVoltage(ConveyorConstants.kConvey);
                kicker.setVoltage(KickerConstants.kKick);
              } else {
                conveyor.setVoltage(ConveyorConstants.kOff);
                kicker.setVoltage(KickerConstants.kOff);
              }
            },
            conveyor, kicker, shooter, hood)
          .finallyDo((interrupted) -> {
              shooter.setReference(0);
              hood.setReference(HoodConstants.kMinPos);
              conveyor.setVoltage(ConveyorConstants.kOff);
              kicker.setVoltage(KickerConstants.kOff);
          }),

        new SequentialCommandGroup(
            new WaitCommand(1.0),
            new RunCommand(
                () -> {
                  intake.setExtenderLowCurrentMode(false);
                  intake.setReference(ExtenderConstants.kStow);
                },
                intake)
        )
    );
  }
}