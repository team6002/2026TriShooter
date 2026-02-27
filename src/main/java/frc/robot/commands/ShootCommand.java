package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.constants.FieldConstants;

// import frc.robot.constants.RobotMode;
// import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(RobotContainer robot) {
        addCommands(
                robot.drive.alignToTarget(() -> FieldConstants.getHubPose()),
                new ParallelDeadlineGroup(
                        // Robot.CURRENT_ROBOT_MODE == RobotMode.REAL ?
                        // robot.superStructure.moveToPose(SuperStructurePose.READY_TO_SHOOT) :
                        new ShootFuelSim(robot.driveSimulation, null, null),
                        robot.drive.alignToTarget(() -> FieldConstants.getHubPose())));
    }
}
