package frc.robot.autos.trench;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.AutoAlignToMiddle;

public class AUTO_MiddleSideHF implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("swipehalfM2", mirrored, robot.drive)
            ,followPath("swipehalfM2", mirrored)
            ,followPath("shootfuelM2", mirrored)
            ,new ShootCommand(robot)
            ,new AutoAlignToMiddle(robot.drive)
        );
    }
}
