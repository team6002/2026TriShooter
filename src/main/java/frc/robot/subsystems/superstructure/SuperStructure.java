package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.*;

public class SuperStructure {
    public enum SuperStructurePose {
        EXTENDED(
            Radians.of(ExtenderConstants.kExtended), // intake extension
            Volts.of(IntakeConstants.kOff), // intake power
            Volts.of(ConveyorConstants.kOff), // conveyor power
            Volts.of(KickerConstants.kOff) // kicker power
        )
        ,HOME(
           Radians.of(ExtenderConstants.kHome), // intake extension
            Volts.of(IntakeConstants.kOff), // intake power
            Volts.of(ConveyorConstants.kOff), // conveyor power
            Volts.of(KickerConstants.kOff) // kicker power 
        )
        ,STOW(
           Radians.of(ExtenderConstants.kStow), // intake extension
            Volts.of(IntakeConstants.kOff), // intake power
            Volts.of(ConveyorConstants.kOff), // conveyor power
            Volts.of(KickerConstants.kOff) // kicker power 
        )
        ,INTAKE(
            Radians.of(ExtenderConstants.kExtended), // intake extension
            Volts.of(IntakeConstants.kOn), // intake power
            Volts.of(ConveyorConstants.kOff), // conveyor power
            Volts.of(KickerConstants.kOff) // kicker power
        )
        ,READY_TO_SHOOT(
            Radians.of(ExtenderConstants.kStow), // intake extension
            Volts.of(IntakeConstants.kOff), // intake power
            Volts.of(ConveyorConstants.kConvey), // conveyor power
            Volts.of(KickerConstants.kKick) // kicker power
        );

        public final Angle intakeAngle;
        public final Voltage intakeVoltage;
        public final Voltage conveyorVoltage;
        public final Voltage kickerVoltage;

        SuperStructurePose(Angle intakeAngle, Voltage intakeVoltage, Voltage conveyorVoltage, Voltage kickerVoltage) {
            this.intakeAngle = intakeAngle;
            this.intakeVoltage = intakeVoltage;
            this.conveyorVoltage = conveyorVoltage;
            this.kickerVoltage = kickerVoltage;
        }
    }

    public static List<PoseLink> LINKS = List.of(
        new PoseLink(SuperStructurePose.EXTENDED, SuperStructurePose.INTAKE)
        ,new PoseLink(SuperStructurePose.EXTENDED, SuperStructurePose.HOME)
        ,new PoseLink(SuperStructurePose.EXTENDED, SuperStructurePose.READY_TO_SHOOT)
        ,new PoseLink(SuperStructurePose.HOME, SuperStructurePose.READY_TO_SHOOT)
        ,new PoseLink(SuperStructurePose.HOME, SuperStructurePose.INTAKE)
        ,new PoseLink(SuperStructurePose.HOME, SuperStructurePose.STOW)
        ,new PoseLink(SuperStructurePose.INTAKE, SuperStructurePose.STOW)
        ,new PoseLink(SuperStructurePose.EXTENDED, SuperStructurePose.STOW)
        ,new PoseLink(SuperStructurePose.READY_TO_SHOOT, SuperStructurePose.STOW)
    );

    public record PoseLink(SuperStructurePose pose1, SuperStructurePose pose2) {
        public double timeSeconds() {
            double intakeDifferenceRad = pose2.intakeAngle.minus(pose1.intakeAngle).abs(Radians);
            double intakeTime = calculateTimeToSetpoint(intakeDifferenceRad, ExtenderConstants.kMaxAccel, ExtenderConstants.kMaxVel);

            return Math.max(intakeTime, 0);
        }

        private static double calculateTimeToSetpoint(double difference, double maxAcc, double maxVel) {
            difference = Math.abs(difference);

            double timeToMaxVel = maxVel / maxAcc;

            double distanceDuringAcc = 0.5 * maxAcc * timeToMaxVel * timeToMaxVel;

            if (difference <= 2 * distanceDuringAcc)
                return 2 * Math.sqrt(difference / maxAcc);

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

    // private final Climb climb;
    private final Conveyor conveyor;
    private final Hood hood;
    private final Intake intake;
    private final Kicker kicker;
    private final Shooter shooter;

    private SuperStructurePose currentPose;
    private SuperStructurePose goal;

    public SuperStructure(Conveyor conveyor, Hood hood, Intake intake, Kicker kicker, Shooter shooter) {
        // this.climb = climb;
        this.conveyor = conveyor;
        this.hood = hood;
        this.intake = intake;
        this.kicker = kicker;
        this.shooter = shooter;

        this.goal = this.currentPose = SuperStructurePose.EXTENDED;
    }

    private Command runPose(SuperStructurePose pose) {
        if(pose == SuperStructurePose.READY_TO_SHOOT){
            return Commands.sequence(
                intake.runVoltage(pose.intakeVoltage.baseUnitMagnitude()),
                hood.setTargetPos(.2),
                shooter.setTargetVelolcity(Math.toRadians(20000)),
                new WaitUntilCommand(()-> shooter.isReady()),
                Commands.runOnce(()-> shooter.startShooting()),
                conveyor.runVoltage(pose.conveyorVoltage.baseUnitMagnitude()),
                kicker.runVoltage(pose.kickerVoltage.baseUnitMagnitude()),
                intake.setExtenderTargetAngle(pose.intakeAngle.in(Radians)),
                Commands.runOnce(()-> currentPose = pose)
            )
            .finallyDo(
                interrupted -> {
                    if(interrupted){
                        shooter.setReference(0);
                        shooter.stopShooting();
                        kicker.setVoltage(KickerConstants.kOff);
                        hood.setTargetPos(HoodConstants.kMinAngle);
                    }
                }
            );
        }

        return Commands.runOnce(
            ()-> {
                shooter.stopShooting();
                conveyor.setVoltage(pose.conveyorVoltage.baseUnitMagnitude());
                // climb.setReference(pose.climbAngle.in(Radians));
                hood.setTargetPos(HoodConstants.kMinAngle);
                intake.setVoltage(pose.intakeVoltage.baseUnitMagnitude());
                intake.setExtenderReference(pose.intakeAngle.in(Radians));
                kicker.setVoltage(pose.kickerVoltage.baseUnitMagnitude());
                currentPose = pose;
            },
            conveyor, intake, kicker
        );
    }

    public Command moveToPose(SuperStructurePose pose) {
        return Commands.defer(() -> generateMoveToPoseCommand(pose), Set.of())
            .beforeStarting(() -> goal = pose);
    }

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

    public static Optional<List<SuperStructurePose>> getTrajectory(
            SuperStructurePose startingPose, SuperStructurePose targetPose) {
        if (startingPose.equals(targetPose)) return Optional.of(new ArrayList<>());

        Set<SuperStructurePose> unvisited = new HashSet<>(Set.of(SuperStructurePose.values()));
        // The path that leads to a node with minimum time
        Map<SuperStructurePose, PoseLink> minimumTimePathToNode = new HashMap<>();
        Map<SuperStructurePose, Double> minimumTimeToPose = new HashMap<>();
        PriorityQueue<PoseLink> linksToExamine = new PriorityQueue<>(Comparator.comparingDouble(PoseLink::timeSeconds));
        for (SuperStructurePose pose : SuperStructurePose.values())
            minimumTimeToPose.put(pose, Double.POSITIVE_INFINITY);
        minimumTimeToPose.put(startingPose, 0.0);

        SuperStructurePose currentNode = startingPose;

        int i;
        // limit loop time to avoid code crashes
        for (i = 0; i < loopNumLimit; i++) {
            for (PoseLink link : LINKS) {
                Optional<SuperStructurePose> otherNode = link.otherEdge(currentNode);
                if (otherNode.isEmpty()) continue;
                linksToExamine.add(link);

                double newTime = minimumTimeToPose.get(currentNode) + link.timeSeconds();
                if (newTime < minimumTimeToPose.get(otherNode.get())) {
                    minimumTimePathToNode.put(otherNode.get(), link);
                    minimumTimeToPose.put(otherNode.get(), newTime);
                }
            }

            unvisited.remove(currentNode);

            // select the next
            PoseLink linkToExamine;
            while ((linkToExamine = linksToExamine.poll()) != null) {
                if (unvisited.contains(linkToExamine.pose1)) {
                    currentNode = linkToExamine.pose1;
                    break;
                }
                if (unvisited.contains(linkToExamine.pose2)) {
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
            if (!minimumTimePathToNode.containsKey(tmp)) {
                DriverStation.reportError("Internal Error while tracing back trajectory", true);
                return Optional.empty();
            }
            Optional<SuperStructurePose> otherEdge =
                    minimumTimePathToNode.get(tmp).otherEdge(tmp);
            if (otherEdge.isEmpty()) {
                DriverStation.reportError("Internal Error while tracing back trajectory", true);
                return Optional.empty();
            }
            tmp = otherEdge.get();
        }

        DriverStation.reportError(
            "Internal Error: destination reached, but cannot traceback trajectory in " + loopNumLimit
                + " iterations",
            true);
        return Optional.empty();
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
        for (int i = 0; i < 10; i++) getTrajectory(SuperStructurePose.EXTENDED, SuperStructurePose.INTAKE);
        System.out.println("tried 10 plans, took " + (System.currentTimeMillis() - t0) + " ms");
    }

    public Command warmUpCommand() {
        return Commands.run(this::testTrajectoryGen)
            .until(DriverStation::isEnabled)
            .withTimeout(10)
            .ignoringDisable(true);
    }
}