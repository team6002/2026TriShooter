// package frc.robot.commands;

// import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision.Vision;

// public class TheAutoAlign extends SequentialCommandGroup {
//     public TheAutoAlign(SwerveDriveSimulation driveSimulation, Vision vision, Drive drive, double tolerance, int cameraIndex, int rotation) {
//         if (vision.lastResult(driveSimulation, cameraIndex) != null) {
//             var targetPose = vision.lastResult(driveSimulation, cameraIndex);
//             addCommands(
//                 //0.2, 0.23
//                 new goToPose(targetPose.plus(new Transform2d(new Translation2d(0.2, 0.23), new Rotation2d(Math.toRadians(rotation)))), driveSimulation, drive, tolerance)
//             );
//         }
//     }

//     public TheAutoAlign(Vision vision, Drive drive, double tolerance, int cameraIndex, int rotation) {
//         if (vision.lastResult(drive, cameraIndex) != null) {
//             var targetPose = vision.lastResult(drive, cameraIndex);
//             addCommands(
//                 new goToPose(targetPose.plus(new Transform2d(new Translation2d(0.2, 0.23), new Rotation2d(Math.toRadians(rotation)))), drive, tolerance)
//             );
//         }
//     }
// }
package frc.robot.commands;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class TheAutoAlign extends SequentialCommandGroup {

    public TheAutoAlign(SwerveDriveSimulation sim, Vision vision, Drive drive,
                        double tolerance, int cameraIndex, int rotationDeg) {

        addCommands(
            new ConditionalCommand(
                // If target exists → run goToPose
                new InstantCommand(() -> {
                    var target = vision.lastResult(sim, cameraIndex);
                    if (target != null) {
                        var offset = new Transform2d(
                            new Translation2d(0, 0),
                            Rotation2d.fromDegrees(rotationDeg)
                        );
                        CommandScheduler.getInstance().schedule(new goToPose(target.plus(offset), drive, tolerance));
                    }
                }),

                // If no target → do nothing
                new InstantCommand(),

                // Condition
                () -> vision.lastResult(sim, cameraIndex) != null
            )
        );
    }

    public TheAutoAlign(Vision vision, Drive drive,
                        double tolerance, int cameraIndex, int rotationDeg) {

        addCommands(
            new ConditionalCommand(
                // If target exists → run goToPose
                new InstantCommand(() -> {
                    var target = vision.lastResult(drive, cameraIndex);
                    if (target != null) {
                        var offset = new Transform2d(
                            new Translation2d(0, 0),
                            Rotation2d.fromDegrees(rotationDeg)
                        );
                        CommandScheduler.getInstance().schedule(new goToPose(target.plus(offset), drive, tolerance));
                    }
                }),

                // If no target → do nothing
                new InstantCommand(),

                // Condition
                () -> vision.lastResult(drive, cameraIndex) != null
            )
        );
    }
}