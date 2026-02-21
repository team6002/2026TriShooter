package frc.robot.autos.trench;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.AutoAlignToMiddle;

public class AUTO_SideFF implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("gotomiddleS1", mirrored, robot.drive)
            ,followPath("gotomiddleS1", mirrored)
            ,followPath("grabfuelS1", mirrored)
            ,followPath("gotolineS1", mirrored)
            ,new ShootCommand(robot)
            ,new AutoAlignToMiddle(robot.drive)
        );
    }
}
