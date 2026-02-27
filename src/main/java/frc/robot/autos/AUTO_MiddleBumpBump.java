package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.AutoAlignToMiddle;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_MiddleBumpBump implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored)
            throws IOException, ParseException {
        return Commands.sequence(
                setAutoStartPose("swipehalfbumpM2", mirrored, robot.drive),
                followPath("swipehalfbumpM2", mirrored),
                followPath("shootfuelbumpM2", mirrored),
                new ShootCommand(robot),
                new AutoAlignToMiddle(robot.drive));
    }
}
