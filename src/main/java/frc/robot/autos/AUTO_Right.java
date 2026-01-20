package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeIOSim;

public class AUTO_Right extends SequentialCommandGroup {
    public AUTO_Right(Drive drive, SwerveDriveSimulation sim, Boolean mirrored) {
        addCommands(
            new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(8))
            ,new InstantCommand(()->drive.setAutoStartPose("getmiddleR1", mirrored))
            ,drive.followPath("getmiddleR1", mirrored)
            ,drive.followPath("gotolineR1", mirrored)
            ,new ParallelCommandGroup(
                drive.followPath("gotoHPR1", mirrored)
                ,new ShootFuelSim(sim)
            )
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,new ParallelCommandGroup(
                drive.followPath("climbshootR1", mirrored)
                ,new ShootFuelSim(sim)
            )
        );
    }
}