package frc.robot.autos.trench;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.AutoAlignToMiddle;

public class AUTO_MiddleRightHF implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("swipehalfM2", false, robot.drive)
            ,followPath("swipehalfM2", false)
            ,followPath("shootfuelM2", false)
            ,new ShootCommand(robot)
            ,new AutoAlignToMiddle(robot.drive)
        );
    }
}
