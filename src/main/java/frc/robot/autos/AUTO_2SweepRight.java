package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class AUTO_2SweepRight implements Auto {
  @Override
  public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
    return Commands.sequence(
        setAutoStartPose("FirstSweep2Sweep", true, robot.drive),
        followPath("FirstSweep2Sweep", true),
        followPath("ShootFirstCycle2Sweep", true),
        robot.shootClose(),
        followPath("SecondSweep2Sweep", true),
        followPath("ShootSecondCycle2Sweep", true),
        robot.shootClose());
  }
}
