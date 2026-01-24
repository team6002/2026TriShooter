package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_SideHump extends SequentialCommandGroup {
    public AUTO_SideHump(Drive drive, SwerveDriveSimulation sim, Boolean mirrored) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8)),
            drive.setAutoStartPose("gotomiddleSH1", mirrored)
            ,drive.followPath("gotomiddleSH1", mirrored)
            ,drive.followPath("grabmiddleSH1", mirrored)
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbSH1", mirrored)
        );
    }
}
