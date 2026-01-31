package frc.robot.autos.hump;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_MiddleHump extends SequentialCommandGroup {
    public AUTO_MiddleHump(Drive drive, SwerveDriveSimulation sim) {
        addCommands(
            drive.setAutoStartPose("gotomiddleM3", false)
            ,drive.followPath("gotomiddleM3", false)
            ,drive.followPath("pickmiddleM3", false)
            ,drive.followPath("gotolineM3", false)
            ,drive.followPath("gotoHPM3", false)
            ,drive.aimAtTarget(FieldConstants.getHubPose().plus(new Translation2d(0, -0.5)))
            ,new ShootFuelSim(sim)
            ,new WaitCommand(1)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new ShootFuelSim(sim)
            ,drive.followPath("climbshootM3", false)
        );
    }
}
