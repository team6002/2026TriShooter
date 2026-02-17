package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootFuelSim;
import frc.robot.commands.drive.AutoAlignToClimb;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;

public class AUTO_MiddleLeftSide implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("swipehalfM2", true, robot.drive)
            ,followPath("swipehalfM2", true)
            ,followPath("shootfuelM2", true)
            ,robot.drive.alignToTarget(()->FieldConstants.getHubPose().plus(new Translation2d(0, 0.5)))
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new ShootFuel(robot.drive, robot.conveyor, robot.intake, null, null, null) : 
                new ShootFuelSim(robot.driveSimulation)
            ,new AutoAlignToClimb(robot.drive)
        );
    }
}
