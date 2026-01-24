package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Right extends SequentialCommandGroup {
    public AUTO_Right(Drive drive, SwerveDriveSimulation sim) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8))
            ,drive.setAutoStartPose("getmiddleR1", false)
            ,drive.followPath("getmiddleR1", false)
            ,drive.followPath("gotolineR1", false)
            ,drive.followPath("gotoHPR1", false)
            ,drive.aimAtTarget(FieldConstants.getHubPose())
            ,new ShootFuelSim(sim)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,drive.followPath("climbshootR1", false)
            ,new ShootFuelSim(sim)
        );
    }
}