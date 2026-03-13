package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_2Sweep implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("FirstSweep2Sweep", false, robot.drive),
        followPath("FirstSweep2Sweep", false),
        followPath("ShootFirstCycle2Sweep", false),
        robot.shootClose(),
        followPath("SecondSweep2Sweep", false),
        followPath("ShootSecondCycle2Sweep", false),
        robot.shootClose());
  }
}