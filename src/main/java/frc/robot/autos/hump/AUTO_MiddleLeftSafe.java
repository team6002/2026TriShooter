package frc.robot.autos.hump;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_MiddleLeftSafe extends SequentialCommandGroup {
    public AUTO_MiddleLeftSafe(Drive drive, SwerveDriveSimulation sim) {
        addCommands(
            drive.setAutoStartPose("gotodepotMLS", false)
            ,drive.followPath("gotodepotMLS", false)
            ,new WaitCommand(0.5)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbMLS", false)
        );
    }
}
