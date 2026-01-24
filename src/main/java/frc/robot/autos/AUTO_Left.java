package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Left extends SequentialCommandGroup {
    public AUTO_Left(Drive drive, Boolean mirrored, SwerveDriveSimulation sim) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8)),
            drive.setAutoStartPose("gotodepotL1", mirrored),
            drive.followPath("gotodepotL1", mirrored),
            new InstantCommand(()->sim.rotateAboutCenter(FieldConstants.getHubPose().minus(drive.getPose().getTranslation()).getAngle().getRadians() + (DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI)))
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new ShootFuelSim(sim)
        );
    }
}
