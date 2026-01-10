package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.*;
import java.util.*;

public class SuperStructure {
    /**
     * Represents a pose of the super structure
     *
     * <p>The pose of the super structure is the combination of the elevator height and arm position
     */
    public enum SuperStructurePose {
        // Useful poses
        IDLE(Inches.of(0), Degrees.of(50), Degrees.of(0), Degrees.of(0), Degrees.of(185)),
        INTERMEDIATE(Inches.of(0), Degrees.of(90), Degrees.of(0), Degrees.of(0), Degrees.of(185)),
        PREPARE_TO_RUN(Inches.of(0), Degrees.of(90), Degrees.of(0), Degrees.of(0), Degrees.of(185)),
        INTAKE_SHELF(Inches.of(0), Degrees.of(50), Degrees.of(-33), Degrees.of(-90), Degrees.of(185)),
        SCORE_L2(Inches.of(0), Degrees.of(50), Degrees.of(-33), Degrees.of(-90), Degrees.of(185)),
        SCORE_L3(Inches.of(0), Degrees.of(50), Degrees.of(-30), Degrees.of(-90), Degrees.of(185)),
        SCORE_L4(Inches.of(0), Degrees.of(50), Degrees.of(-30), Degrees.of(-90), Degrees.of(185));

        public final Distance elevatorHeightMeters;
        public final Angle pivotAngle, wristAngle, twistAngle, intakeAngle;

        SuperStructurePose(
                Distance elevatorHeightMeters,
                Angle pivotAngle,
                Angle wristAngle,
                Angle twistAngle,
                Angle IntakeAngle) {
            this.elevatorHeightMeters = elevatorHeightMeters;
            this.pivotAngle = pivotAngle;
            this.wristAngle = wristAngle;
            this.twistAngle = twistAngle;
            this.intakeAngle = IntakeAngle;
        }
    }

    public static List<PoseLink> LINKS = List.of(
            // We can run to intake / l2 / l3 directly from idle
            new PoseLink(SuperStructurePose.IDLE, SuperStructurePose.INTERMEDIATE),
            new PoseLink(SuperStructurePose.PREPARE_TO_RUN, SuperStructurePose.INTERMEDIATE),
            // new PoseLink(SuperStructurePose.PREPARE_TO_RUN, SuperStructurePose.IDLE),
            // new PoseLink(SuperStructurePose.PREPARE_TO_RUN, SuperStructurePose.SCORE_L2),
            // new PoseLink(SuperStructurePose.PREPARE_TO_RUN, SuperStructurePose.SCORE_L3),
            new PoseLink(SuperStructurePose.SCORE_L2, SuperStructurePose.SCORE_L3));

    /**
     * Represents a link between two super structure poses
     *
     * <p>If two poses are linked, it means that the super structure can move from pose1 to pose2 directly (without
     * hitting dead-bands).
     */
    public record PoseLink(SuperStructurePose pose1, SuperStructurePose pose2) {
        /** Calculates the amount of time needed for the super structure to move */
        public double timeSeconds() {
            // Differences between the current and target poses
            double pivotDifferenceRad = pose2.pivotAngle.minus(pose1.pivotAngle).abs(Radians);
            double elevatorDifferenceM = Math.abs(
                    pose1.elevatorHeightMeters.minus(pose2.elevatorHeightMeters).in(Meters));

            // Constraints from the constants file
            double pivotMaxAcc = PivotConstants_MainPivot.m_MaxAcceleration.in(RadiansPerSecondPerSecond);
            double pivotMaxVel = PivotConstants_MainPivot.m_MaxVelocity.in(RadiansPerSecond);
            double elevatorAcc = ElevatorConstants.m_MaxAcceleration.in(MetersPerSecondPerSecond);
            double elevatorVel = ElevatorConstants.m_MaxVelocity.in(MetersPerSecond);

            // Time to move arm
            double pivotTime = calculateTimeToSetpoint(pivotDifferenceRad, pivotMaxAcc, pivotMaxVel);

            // Time to move elevator
            double elevatorTime = calculateTimeToSetpoint(elevatorDifferenceM, elevatorAcc, elevatorVel);

            // The total time is the maximum of the arm and elevator times
            // return Math.max(armTime, elevatorTime);
            return Math.max(pivotTime, elevatorTime);
        }

        /** Helper method to calculate time needed for a mechanism to move to a setpoint Author: ChatGPT-o3-mini-high */
        private static double calculateTimeToSetpoint(double difference, double maxAcc, double maxVel) {
            // Take the absolute value of difference to prevent negative values
            difference = Math.abs(difference);

            // Time to reach max velocity during acceleration phase
            double timeToMaxVel = maxVel / maxAcc;

            // Distance covered during acceleration to max velocity
            double distanceDuringAcc = 0.5 * maxAcc * timeToMaxVel * timeToMaxVel;

            // If the difference can be covered during acceleration and deceleration
            if (difference <= 2 * distanceDuringAcc)
                // We won't reach max velocity, so use the equation for constant acceleration/deceleration
                return 2 * Math.sqrt(difference / maxAcc);

            // Time to accelerate and decelerate
            double distanceAtConstantVelocity = difference - 2 * distanceDuringAcc;
            double timeAtConstantVel = distanceAtConstantVelocity / maxVel;

            return 2 * timeToMaxVel + timeAtConstantVel;
        }

        public Optional<SuperStructurePose> otherEdge(SuperStructurePose oneEdge) {
            if (oneEdge == pose1) return Optional.of(pose2);
            if (oneEdge == pose2) return Optional.of(pose1);
            else return Optional.empty();
        }
    }

    private final Elevator elevator;
    private final Pivot pivot, wristPivot, twistPivot, intakePivot;

    private SuperStructurePose currentPose;
    private SuperStructurePose goal;
    public final Trigger atReference;

    public SuperStructure(Elevator elevator, Pivot pivot, Pivot wristPivot, Pivot twistPivot, Pivot intakePivot) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wristPivot = wristPivot;
        this.twistPivot = twistPivot;
        this.intakePivot = intakePivot;
        this.goal = this.currentPose = SuperStructurePose.IDLE;

        atReference = new Trigger(() -> elevator.atReference(goal.elevatorHeightMeters)
                && pivot.atReference(goal.pivotAngle)
                && wristPivot.atReference(goal.wristAngle)
                && twistPivot.atReference(goal.twistAngle)
                && intakePivot.atReference(goal.intakeAngle));

        atReference.onTrue(Commands.runOnce(() -> currentPose = goal));

        new Trigger(DriverStation::isTeleop).onTrue(moveToPose(SuperStructurePose.IDLE));

        warmUpCommand().schedule();
    }

    private Command runPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorHeightMeters)
                .alongWith(pivot.moveToPosition(pose.pivotAngle))
                .alongWith(wristPivot.moveToPosition(pose.wristAngle))
                .alongWith(twistPivot.moveToPosition(pose.twistAngle))
                .alongWith(intakePivot.moveToPosition(pose.intakeAngle))
                .beforeStarting(Commands.print("Super Structure/Moving to pose: " + pose.name()))
                .finallyDo(interrupted -> {
                    currentPose = pose;
                    if (interrupted)
                        System.out.println("Super Structure/Interrupted while running to pose: " + pose.name());
                    else System.out.println("Super Structure/Reached pose: " + pose.name());
                });
    }

    public Command moveToPose(SuperStructurePose pose) {
        return Commands.defer(() -> generateMoveToPoseCommand(pose), Set.of(elevator, pivot))
                .beforeStarting(() -> goal = pose);
    }

    // public Command retrieveElevator() {
    //     return Commands.deferredProxy(() -> moveToPose(
    //             switch (targetPose()) {
    //                 case PREPARE_TO_RUN,
    //                         SCORE_L2,
    //                         SCORE_L3,
    //                         SCORE_L4,
    //                         SCORE_L4_COMPLETE,
    //                         LOW_SWAP_1,
    //                         LOW_SWAP_2,
    //                         HIGH_SWAP,
    //                         ALGAE_SWAP_1,
    //                         ALGAE_SWAP_3 -> SuperStructure.SuperStructurePose.PREPARE_TO_RUN;
    //                 case PREPARE_TO_GRAB_LOW_ALGAE,
    //                         PREPARE_TO_GRAB_HIGH_ALGAE,
    //                         GRAB_LOW_ALGAE,
    //                         GRAB_HIGH_ALGAE,
    //                         SCORE_ALGAE,
    //                         ALGAE_SWAP_2,
    //                         ALGAE_SWAP_4 -> SuperStructure.SuperStructurePose.SCORE_ALGAE;
    //                 case IDLE -> SuperStructure.SuperStructurePose.IDLE;
    //             }));
    // }

    private Command generateMoveToPoseCommand(SuperStructurePose pose) {
        Optional<List<SuperStructurePose>> trajectory = getTrajectory(pose);
        if (trajectory.isEmpty()) return this.runPose(pose);
        return Commands.sequence(trajectory.get().stream().map(this::runPose).toArray(Command[]::new));
    }

    public SuperStructurePose currentPose() {
        return currentPose;
    }

    public SuperStructurePose targetPose() {
        return goal;
    }

    private static final int loopNumLimit = 100;
    /**
     * Finds the minimum-time trajectory to move from a pose to a target pose.
     *
     * <p>This uses a shortest path algorithm (Dijkstra) to find the optimal path considering the time to move between
     * poses.
     */
    public static Optional<List<SuperStructurePose>> getTrajectory(
            SuperStructurePose startingPose, SuperStructurePose targetPose) {
        if (startingPose.equals(targetPose)) return Optional.of(new ArrayList<>());

        // create a set of poses
        Set<SuperStructurePose> unvisited = new HashSet<>(Set.of(SuperStructurePose.values()));
        // The path that leads to a node with minimum time
        Map<SuperStructurePose, PoseLink> minimumTimePathToNode = new HashMap<>();
        Map<SuperStructurePose, Double> minimumTimeToPose = new HashMap<>();
        PriorityQueue<PoseLink> linksToExamine = new PriorityQueue<>(Comparator.comparingDouble(PoseLink::timeSeconds));
        for (SuperStructurePose pose : SuperStructurePose.values())
            minimumTimeToPose.put(pose, Double.POSITIVE_INFINITY);
        minimumTimeToPose.put(startingPose, 0.0);

        SuperStructurePose currentNode = startingPose; // use the startingPose as the first node to examine
        int i;
        // limit loop time to avoid code crashes
        for (i = 0; i < loopNumLimit; i++) {
            // traverse thru all the defined links
            for (PoseLink link : LINKS) {
                // check to see if the link matches the currentNode
                Optional<SuperStructurePose> otherNode =
                        link.otherEdge(currentNode); // return the other node if the current link matches
                if (otherNode.isEmpty()) continue; // empty = no match, skip to the next for loop element
                // proceed because we found a link
                linksToExamine.add(link); // add to the list of nodes to examine
                double newTime = minimumTimeToPose.get(currentNode)
                        + link.timeSeconds(); // calculate the cumulative time for the node
                if (newTime < minimumTimeToPose.get(otherNode.get())) { // if this the short time to get to this node?
                    // if yes, save the node to represent the shortest previous path node
                    minimumTimePathToNode.put(otherNode.get(), link);
                    minimumTimeToPose.put(otherNode.get(), newTime);
                }
            }

            unvisited.remove(currentNode);

            // select the next
            PoseLink linkToExamine;
            while ((linkToExamine = linksToExamine.poll())
                    != null) { // try to get a node from the list of links to examine
                if (unvisited.contains(linkToExamine.pose1)) { // if the pose1 has not been visited, try it next
                    currentNode = linkToExamine.pose1;
                    break;
                }
                if (unvisited.contains(linkToExamine.pose2)) { // if the pose2 has not been visited, try it next
                    currentNode = linkToExamine.pose2;
                    break;
                }
            }
            if (linkToExamine == null) break;
        }

        List<SuperStructurePose> trajectory = new ArrayList<>();
        SuperStructurePose tmp = targetPose;
        System.out.println("<-- tracing trajectory: -->");
        for (int j = 0; j < loopNumLimit; j++) {
            System.out.println("    tracing node: " + tmp);
            trajectory.add(0, tmp);
            if (tmp == startingPose) {
                System.out.println("Successfully planned trajectory: " + printTrajectory(trajectory));
                return Optional.of(trajectory);
            }
            // error checking, pose1 should not contain the current node.
            // if true, it should have already exit loop as the complete trajetory
            if (!minimumTimePathToNode.containsKey(tmp)) {
                DriverStation.reportError("Internal Error while tracing back trajectory", true);
                return Optional.of(new ArrayList<>());
            }
            // get the other node of the link, this should be our next trajetory
            Optional<SuperStructurePose> otherEdge =
                    minimumTimePathToNode.get(tmp).otherEdge(tmp);
            if (otherEdge.isEmpty()) { // This should not be empty. something is wrong if its empty
                DriverStation.reportError("Internal Error while tracing back trajectory", true);
                return Optional.of(new ArrayList<>());
            }
            tmp = otherEdge.get();
        }

        DriverStation.reportError(
                "Internal Error: destination reached, but cannot traceback trajectory in " + loopNumLimit
                        + " iterations",
                true);
        return Optional.of(new ArrayList<>());
    }

    private static String printTrajectory(List<SuperStructurePose> trajectory) {
        if (trajectory.isEmpty()) return "(Empty Trajectory)";
        StringBuilder message = new StringBuilder();
        for (int i = 0; i < trajectory.size() - 1; i++)
            message.append(trajectory.get(i).name()).append(" -> ");
        message.append(trajectory.get(trajectory.size() - 1).name());
        return message.toString();
    }

    public Optional<List<SuperStructurePose>> getTrajectory(SuperStructurePose targetPose) {
        return getTrajectory(currentPose, targetPose);
    }

    private void testTrajectoryGen() {
        long t0 = System.currentTimeMillis();
        for (int i = 0; i < 10; i++) getTrajectory(SuperStructurePose.IDLE, SuperStructurePose.SCORE_L4);
        System.out.println("tried 10 plans, took " + (System.currentTimeMillis() - t0) + " ms");
    }

    public Command warmUpCommand() {
        return Commands.run(this::testTrajectoryGen)
                .until(DriverStation::isEnabled)
                .withTimeout(10)
                .ignoringDisable(true);
    }
}
