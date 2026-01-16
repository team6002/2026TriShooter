package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class AUTO_Left extends SequentialCommandGroup {
    public AUTO_Left(Drive drive) {
        addCommands(
            new InstantCommand(()->drive.setAutoStartPose("gotomiddle"))
            ,drive.followPath("gotomiddle")
            ,drive.followPath("grabmiddlefuel")
            ,drive.followPath("scoreitall")
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