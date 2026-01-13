package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class AUTO_Middle extends SequentialCommandGroup {
    public AUTO_Middle(Drive drive) {
        addCommands(
            new InstantCommand(()->drive.setAutoStartPose("pickupfromHP"))
            ,drive.followPath("pickupfromHP")
            ,new WaitCommand(2)
            ,drive.followPath("shootfirstcycle")
            ,new WaitCommand(6)
            // ,drive.followPath("pickuplastcycle")
            // ,drive.followPath("shootlastcycle")
            // ,new WaitCommand(1.25)
            ,drive.followPath("climb")
        );
    }
}