package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootFuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Middle extends SequentialCommandGroup {
    public AUTO_Middle(Drive drive, SwerveDriveSimulation sim) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8))
            ,new InstantCommand(()->drive.setAutoStartPose("pickupfromHP"))
            ,drive.followPath("pickupfromHP")
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new WaitCommand(2)
            // ,drive.followPath("shootfirstcycle")
            ,new InstantCommand(()->sim.rotateAboutCenter(Math.atan2(4-drive.getPose().getY(), 4-drive.getPose().getX())))
            ,new ShootFuelSim(sim)
            ,drive.followPath("pickuplastcycle")
            ,drive.followPath("shootlastcycle")
            ,new ShootFuelSim(sim)
            ,drive.followPath("climb")
        );
    }
}