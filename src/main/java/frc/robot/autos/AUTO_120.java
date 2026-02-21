package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.AutoAlignToMiddle;

public class AUTO_120 implements Auto{
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("SwipeHalfMiddle", false, robot.drive)
            ,followPath("SwipeHalfMiddle", false)
            ,followPath("Shoot120", false)
            ,new ShootCommand(robot)
        //     ,followPath("SwipeMiddle", false)
        //     ,followPath("Shoot120Bottom", false)
        //     ,new ShootCommand(robot)
        //     ,new AutoAlignToMiddle(robot.drive)
        );
    }
}