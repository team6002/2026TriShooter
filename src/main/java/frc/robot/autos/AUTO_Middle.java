package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Middle extends SequentialCommandGroup {
    public AUTO_Middle(Drive drive, SwerveDriveSimulation sim, Boolean mirrored) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8))
            ,drive.setAutoStartPose("pickupHPM1", mirrored)
            ,drive.followPath("pickupHPM1", mirrored)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new WaitCommand(2)
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,drive.followPath("pickupmiddleM1", mirrored)
            ,drive.followPath("shootclimbM1", mirrored)
            ,new ShootFuelSim(sim)
        );
    }
}