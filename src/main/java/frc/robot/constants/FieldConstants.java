package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class FieldConstants {
        public static final Translation2d BlueHubPose = new Translation2d(Units.inchesToMeters(182), Units.inchesToMeters(159));
        public static final Translation2d RedHubPose = new Translation2d(Units.inchesToMeters(469), Units.inchesToMeters(159));
        
        public static DriverStation.Alliance Alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        public static Translation2d HubPose = (Alliance == DriverStation.Alliance.Blue) ? BlueHubPose : RedHubPose;

        public static final Distance ROBOT_TO_TARGET_DISTANCE = Robot.CURRENT_ROBOT_MODE == RobotMode.SIM
        // for simulation
        ? Centimeters.of(37.3)
        // for real robot (measure this on field)
        // "AdvantageKit/RealOutputs/RobotToSelectedBranchTarget" - X Axis
        : Centimeters.of(40.0);

        public static void updateAlliance(){
                Alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                HubPose = (Alliance == DriverStation.Alliance.Blue) ? BlueHubPose : RedHubPose;
        }
}
