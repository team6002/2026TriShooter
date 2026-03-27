package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

// Assuming 'Auto' is an interface you created
public class AUTO_onthefly implements Auto {
    
    public PathPlannerPath getPath() {
        // 1. Create a MUTABLE list of waypoints
        List<Waypoint> waypoints = new ArrayList<>(PathPlannerPath.waypointsFromPoses(
                new Pose2d(2, 7, Rotation2d.fromDegrees(90)),
                new Pose2d(3, 3, Rotation2d.fromDegrees(-90)),
                new Pose2d(1.5, 2, Rotation2d.fromDegrees(45))
        ));

        // 2. Perform manipulations inside a method
        Random rnd = new Random();
        for (int i = waypoints.size() - 1; i > 0; i--) {
            int index = rnd.nextInt(i + 1);
            Waypoint a = waypoints.get(index);
            waypoints.set(index, waypoints.get(i));
            waypoints.set(i, a);
        }

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

        // 3. Create path
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null
                ,new GoalEndState(0.0, Rotation2d.fromDegrees(-90))
        );
        return path;
    }

    @Override
public Command getAutoCommand(RobotContainer robot) {
    // 1. Generate the path ONCE so the odometry reset and follow command use the same data
    PathPlannerPath path = getPath();

    // 2. Prevent the path from being flipped by alliance (optional, but good for debugging)
    // path.preventFlipping = true; 

    return Commands.sequence(
        // Use the actual path's initial pose to reset
        Commands.runOnce(() -> robot.drive.resetOdometry(path.getStartingHolonomicPose().get()), robot.drive),
        
        // Follow the path
        AutoBuilder.followPath(path)
    );
}
}
