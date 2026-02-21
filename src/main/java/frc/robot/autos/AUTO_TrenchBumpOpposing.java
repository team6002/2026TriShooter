package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class AUTO_TrenchBumpOpposing implements Auto{
    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            //reset odometry and put intake down
            setAutoStartPose("SwipeHalfMiddleTrench", mirrored, robot.drive)
            ,robot.superStructure.moveToPose(SuperStructurePose.INTAKE)
            //run out and intake half of our side of the field
            ,followPath("SwipeHalfMiddleTrench", mirrored)
            ,followPath("ShootTrenchBump", mirrored)
            //shoot fuel
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT_120)
                : new ShootFuelSim(robot.driveSimulation)
            //put intake down and swipe the second half of our side of the field
            ,robot.superStructure.moveToPose(SuperStructurePose.INTAKE)
            ,followPath("SwipeOpposingHalfTrenchBump", mirrored)
            ,followPath("ShootTrenchBumpOpposing", mirrored)
            //shoot fuel
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT_120)
                : new ShootFuelSim(robot.driveSimulation)
        );
    }
}
