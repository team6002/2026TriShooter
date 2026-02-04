package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;

public class AUTO_Right implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            // Commands.runOnce(()-> robot.drive.setPose(getStartingPoseAtBlueAlliance()))
            followPath("getmiddleR1")
            ,followPath("gotolineR1")
            ,followPath("gotoHPR1")
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
            ,followPath("climbshootR1")
        );
    }
}
