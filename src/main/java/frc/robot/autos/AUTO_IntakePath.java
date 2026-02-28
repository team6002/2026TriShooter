package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.utils.constants.RobotMode;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_IntakePath implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
                // reset odometry and put intake down
                setAutoStartPose("IntakePath", mirrored, robot.drive),
                Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? new CMD_Intake(robot.intake) : Commands.none(),
                // run out and intake half of our side of the field
                followPath("IntakePath", mirrored));
    }
}
