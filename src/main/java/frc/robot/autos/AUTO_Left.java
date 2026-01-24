package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.commands.drive.AimAtTarget;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Left extends SequentialCommandGroup {
    public AUTO_Left(Drive drive, SwerveDriveSimulation sim, Boolean safe) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8))
        );

        if (safe) {
            addCommands(
                drive.setAutoStartPose("gotodepotL1", false)
            );
        } else {
            addCommands(
                drive.setAutoStartPose("gotomiddleL1", false)
                ,drive.followPath("gotomiddleL1", false)
                ,drive.followPath("grabmiddleL1", false)
                ,drive.followPath("gotostartL1", false)
            );
        }

        addCommands(
            drive.followPath("gotodepotL1", false)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new AimAtTarget(drive, FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbL1", false)
        );
    }
}
