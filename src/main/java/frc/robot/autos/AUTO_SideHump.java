package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class AUTO_SideHump extends SequentialCommandGroup {
    public AUTO_SideHump(Drive drive, Boolean mirrored, SwerveDriveSimulation sim) {
        addCommands(
            drive.setAutoStartPose("gotomiddleSH1", mirrored)
            ,drive.followPath("gotomiddleSH1", mirrored)
            ,drive.followPath("grabmiddleSH1", mirrored)
            ,new InstantCommand(()->sim.rotateAboutCenter(FieldConstants.getHubPose().minus(drive.getPose().getTranslation()).getAngle().getRadians() + (DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI)))
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbSH1", mirrored)
        );
    }
}
