package frc.robot.autos.hump;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_MiddleHumpFull extends SequentialCommandGroup {
    public AUTO_MiddleHumpFull(Drive drive, Boolean mirrored, SwerveDriveSimulation sim) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.setFuelInHopper(8))
            ,drive.setAutoStartPose("gotomiddleMH1", mirrored)
            ,drive.followPath("gotomiddleMH1", mirrored)
            ,drive.followPath("grabmiddleSH1", mirrored)
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbSH1", mirrored)
        );
    }
}
