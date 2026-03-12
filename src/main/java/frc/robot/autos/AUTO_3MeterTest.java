package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_3MeterTest implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
<<<<<<< HEAD
        setAutoStartPose("3MeterTest", mirrored, robot.drive), followPath("3MeterTest", mirrored));
=======
        setAutoStartPose("3MeterTest", false, robot.drive), followPath("3MeterTest", false));
>>>>>>> 168c2bec1ee4f1091547dd1ba9d4c7437c0ed79d
  }
}
