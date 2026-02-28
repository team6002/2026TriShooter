package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
// import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.utils.constants.RobotMode;

// import frc.robot.constants.RobotMode;
// import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(RobotContainer robot) {
        addCommands(
                // robot.drive.alignToTarget(() -> FieldConstants.getHubPose()),
                // new ParallelDeadlineGroup(
                Robot.CURRENT_ROBOT_MODE == RobotMode.REAL
                        ? new CMD_Shoot(
                                        robot.conveyor,
                                        robot.hood,
                                        robot.intake,
                                        robot.kicker,
                                        robot.shooter,
                                        0.35,
                                        Math.toRadians(21000))
                                .withTimeout(5)
                        : new ShootFuelSim(robot.driveSimulation, robot.hood, robot.shooter)
                // robot.drive.alignToTarget(() -> FieldConstants.getHubPose()))
                );
    }
}
