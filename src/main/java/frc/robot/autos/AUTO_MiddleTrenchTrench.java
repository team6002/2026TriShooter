package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.drive.AutoAlignToMiddle;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_MiddleTrenchTrench implements Auto {
    public Command getAutoCommand(RobotContainer robot, boolean mirrored)
            throws IOException, ParseException {
        return Commands.sequence(
                setAutoStartPose("swipehalftrenchM2", mirrored, robot.drive),
                followPath("swipehalftrenchM2", mirrored),
                followPath("shootfueltrenchM2", mirrored),
                new ShootCommand(robot),
                new AutoAlignToMiddle(robot.drive));
    }
}
