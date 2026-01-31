package frc.robot.autos.hump;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_LeftSmall extends SequentialCommandGroup {
    public AUTO_LeftSmall(Drive drive, SwerveDriveSimulation sim) {
        addCommands(
            drive.setAutoStartPose("gotodepotL1", false)
            ,drive.followPath("gotodepotL1", false)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbL1", false)
        );
    }
}
