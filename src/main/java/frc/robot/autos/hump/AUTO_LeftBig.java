package frc.robot.autos.hump;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.Auto;
import frc.robot.commands.ShootFuelSim;
import frc.robot.commands.drive.AutoAlignToClimb;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class AUTO_LeftBig implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("gotomiddleL1", false, robot.drive)
            ,followPath("gotomiddleL1", false)
            ,followPath("gotostartL1", false)
            ,followPath("gotoshootL1", false)
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,new ParallelDeadlineGroup(
                Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT) :
                new ShootFuelSim(robot.driveSimulation)
                ,robot.drive.alignToTarget(()->FieldConstants.getHubPose())
            )
            ,followPath("gotodepotL1", false)
            ,new InstantCommand(()->IntakeIOSim.putFuelInHopperSim(24))
            ,robot.drive.alignToTarget(()-> FieldConstants.getHubPose())
            ,new ParallelDeadlineGroup(
                Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT) :
                new ShootFuelSim(robot.driveSimulation)
                ,robot.drive.alignToTarget(()->FieldConstants.getHubPose())
            )
            ,new AutoAlignToClimb(robot.drive)
        );
    }
}
