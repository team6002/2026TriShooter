package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class AUTO_Middle implements Auto{
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            // Commands.runOnce(()-> robot.drive.setPose(getStartingPoseAtBlueAlliance()))
            followPath("pickupHPM1", false)
            ,followPath("shootfirstcycle", false)
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,new ParallelDeadlineGroup(
                Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT) :
                new ShootFuelSim(robot.driveSimulation)
                ,robot.drive.alignToTarget(()->FieldConstants.getHubPose())
            )
            ,followPath("pickupmiddleM1", false)
            ,followPath("shootclimbM1", false)
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,new ParallelDeadlineGroup(
                Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT) :
                new ShootFuelSim(robot.driveSimulation)
                ,robot.drive.alignToTarget(()->FieldConstants.getHubPose())
            )
        );
    }
}
