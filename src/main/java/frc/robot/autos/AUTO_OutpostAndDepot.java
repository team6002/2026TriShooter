package frc.robot.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootFuelSim;
import frc.robot.constants.RobotMode;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class AUTO_OutpostAndDepot implements Auto{

    @Override
    public Command getAutoCommand(RobotContainer robot, boolean mirrored) throws IOException, ParseException {
        return Commands.sequence(
            setAutoStartPose("IntakeOutpost", mirrored, robot.drive)
            ,followPath("IntakeOutpost", mirrored)
            ,new WaitCommand(3)
            ,followPath("ShootOutpost", mirrored)
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT)
                : new ShootFuelSim(robot.driveSimulation)
            ,followPath("IntakeDepotFromOutpost", mirrored)
            ,followPath("ShootFromDepot", mirrored)
            ,Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ? 
                robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT)
                : new ShootFuelSim(robot.driveSimulation)
        );
    }
}