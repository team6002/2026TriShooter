package frc.robot.autos;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootFuelSim;
import frc.robot.subsystems.drive.Drive;

public class AUTO_Side extends SequentialCommandGroup {
    public AUTO_Side(Drive drive, SwerveDriveSimulation sim, Boolean mirrored) {
        addCommands(
            new InstantCommand(()->drive.setAutoStartPose("gotomiddleL1", mirrored))
            ,drive.followPath("gotomiddleL1", mirrored)
            ,drive.followPath("grabfuelL1", mirrored)
            ,drive.followPath("gotolineL1", mirrored)
            ,new ParallelCommandGroup(
                drive.followPath("climbshootR1", mirrored)
                ,new ShootFuelSim(sim)
            )
        );

        // addCommands(
        //     new InstantCommand(()->drive.setAutoStartPose("gotomiddle"))
        //     ,drive.followPath("gotomiddle")
        //     ,new WaitCommand(2)
        //     ,drive.followPath("grabmiddlefuel")
        //     ,new WaitCommand(6)
        //     // ,drive.followPath("pickuplastcycle")
        //     // ,drive.followPath("shootlastcycle")
        //     // ,new WaitCommand(1.25)
        //     ,drive.followPath("scoreitall")
        // );
    }
}