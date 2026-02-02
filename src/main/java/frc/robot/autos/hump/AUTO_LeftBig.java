package frc.robot.autos.hump;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_LeftBig extends SequentialCommandGroup {
    public AUTO_LeftBig(Drive drive, SwerveDriveSimulation sim) {
        addCommands(
            drive.setAutoStartPose("gotomiddleL1", false)
            ,drive.followPath("gotomiddleL1", false)
            ,drive.followPath("grabmiddleL1", false)
            ,drive.followPath("gotostartL1", false)
            ,drive.followPath("gotodepotL1", false)
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbL1", false)
        );
    }
}
