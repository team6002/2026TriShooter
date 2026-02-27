package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.utils.constants.RobotMode;

public class AUTO_Trench implements Auto{
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            //reset odometry and put intake down
            setAutoStartPose("SwipeHalfMiddleTrench", mirrored, robot.drive)
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new CMD_Intake(robot.intake)
                : Commands.none()
            //run out and intake half of our side of the field
            ,followPath("SwipeHalfMiddleTrench", mirrored)
            ,followPath("ShootTrench", mirrored)
            //shoot fuel
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new CMD_Shoot(robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter, 0.2, Math.toRadians(18000)).withTimeout(5)
                : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter)
            //put intake down and swipe the second half of our side of the field
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new CMD_Intake(robot.intake)
                : Commands.none()
            ,followPath("SwipeMiddleTrench", mirrored)
            ,followPath("ShootBottomTrench", mirrored)
            //shoot fuel
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                new CMD_Shoot(robot.conveyor, robot.hood, robot.intake, robot.kicker, robot.shooter, 0.2, Math.toRadians(18000)).withTimeout(5)
                : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter)
        );
    }
}
