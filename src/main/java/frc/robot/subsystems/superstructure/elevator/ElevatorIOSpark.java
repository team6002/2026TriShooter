package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
    ElevatorProperties m_elevatorProperties;
    public boolean configurationFailed = false;
    private boolean m_hasfollowMotor = false;
    private Distance m_setpoint;

    private TrapezoidProfile m_profile;
    private Timer m_timer;
    private TrapezoidProfile.State m_startState;
    private TrapezoidProfile.State m_endState;
    private TrapezoidProfile.State m_targetState;
    double m_feedforward;
    private double m_manualValue;

    public static TrapezoidProfile.Constraints kArmMotionConstraint;
    public final ElevatorFeedforward kElevatorFeedforward;
    // Hardware objects
    private final SparkMax m_primaryMotor;
    private final SparkMax m_followerMotor;

    private final RelativeEncoder m_relativeEncoder;
    private final SparkClosedLoopController m_motorController;
    // Connection debouncers
    private final Debouncer primaryConnectedDebounce = new Debouncer(0.5);
    // private final Debouncer absoluteEncoderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOSpark(ElevatorProperties elevatorProperties) {
        m_elevatorProperties = elevatorProperties;
        kElevatorFeedforward = elevatorProperties.getArmFeedforward();
        kArmMotionConstraint = new TrapezoidProfile.Constraints(
                elevatorProperties.kMaxVelocity.in(MetersPerSecond),
                elevatorProperties.kMaxAcceleration.in(MetersPerSecondPerSecond));
        // Hardware Creation
        m_primaryMotor = new SparkMax(elevatorProperties.primaryCanID, MotorType.kBrushless);
        m_relativeEncoder = m_primaryMotor.getEncoder();
        m_motorController = m_primaryMotor.getClosedLoopController();

        // Configure Motor
        sparkStickyFault = false;
        tryUntilOk(
                m_primaryMotor,
                5,
                () -> m_primaryMotor.configure(
                        elevatorProperties.getPrimaryConfig(),
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        if (elevatorProperties.hasFollowerMotor) m_hasfollowMotor = true;
        if (m_hasfollowMotor) {
            m_followerMotor = new SparkMax(elevatorProperties.followerCanID, MotorType.kBrushless);
            tryUntilOk(
                    m_followerMotor,
                    5,
                    () -> m_followerMotor.configure(
                            elevatorProperties.getFollowerConfig(),
                            ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters));
        } else {
            m_followerMotor = null;
        }

        configurationFailed = sparkStickyFault;

        m_setpoint = getElevatorHeight();
        m_timer = new Timer();
        m_timer.start();
        updateMotionProfile();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.configurationFailed = this.configurationFailed;
        inputs.hasFollowerMotor = this.m_hasfollowMotor;
        inputs.setPoint = m_setpoint;
        inputs.manualValue = m_manualValue;
        // Obtain encoder Readings
        inputs.currentHeight = getElevatorHeight();
        sparkStickyFault = false;
        ifOk(m_primaryMotor, m_relativeEncoder::getVelocity, (value) -> inputs.velocity = MetersPerSecond.of(value));
        // inputs.absoluteEncoderConnected = absoluteEncoderConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(
                m_primaryMotor,
                new DoubleSupplier[] {m_primaryMotor::getAppliedOutput, m_primaryMotor::getBusVoltage},
                (values) -> inputs.primaryMotorOutputVolts = values[0] * values[1]);
        ifOk(m_primaryMotor, m_primaryMotor::getOutputCurrent, (value) -> inputs.primaryMotorSupplyCurrentAmps = value);
        inputs.primaryMotorConnected = primaryConnectedDebounce.calculate(!sparkStickyFault);

        if (m_hasfollowMotor) {
            sparkStickyFault = false;
            ifOk(
                    m_followerMotor,
                    new DoubleSupplier[] {m_followerMotor::getAppliedOutput, m_followerMotor::getBusVoltage},
                    (values) -> inputs.followerMotorOutputVolts = values[0] * values[1]);
            ifOk(
                    m_followerMotor,
                    m_followerMotor::getOutputCurrent,
                    (value) -> inputs.followerMotorSupplyCurrentAmps = value);
            inputs.followerMotorConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
        }
    }

    @Override
    public void runAutomatic() {
        double elapsedTime = m_timer.get();

        if (m_profile.isFinished(elapsedTime)) {
            m_targetState = new TrapezoidProfile.State(m_setpoint.in(Meters), 0.0);
        } else {
            m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
        }

        m_feedforward =
                kElevatorFeedforward.calculateWithVelocities(m_relativeEncoder.getVelocity(), m_targetState.velocity);
        double _position = m_targetState.position + m_elevatorProperties.kOffsetDistance.in(Meter);
        m_motorController.setReference(_position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward);
    }

    @Override
    public void runManual(double _power) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
        // passively
        m_setpoint = getElevatorHeight();
        updateMotionProfile();
        // update the feedforward variable with the newly zero target velocity
        m_feedforward =
                kElevatorFeedforward.calculateWithVelocities(m_relativeEncoder.getVelocity(), m_targetState.velocity);
        // set the power of the motor
        m_primaryMotor.set(_power + (m_feedforward / 12.0));
        m_manualValue = _power; // this variable is only used for logging or debugging if needed
    }

    @Override
    public void setTargetPosition(Distance _setpoint) {
        _setpoint.plus(m_elevatorProperties.kOffsetDistance);
        if (_setpoint != m_setpoint) {
            m_setpoint = _setpoint;
            updateMotionProfile();
        }
    }

    private void updateMotionProfile() {
        m_startState = new TrapezoidProfile.State(getElevatorHeight().in(Meters), m_relativeEncoder.getVelocity());
        m_endState = new TrapezoidProfile.State(m_setpoint.in(Meters), 0.0);
        m_profile = new TrapezoidProfile(kArmMotionConstraint);
        m_timer.reset();
    }

    public Distance getElevatorHeight() {
        double _distance = m_relativeEncoder.getPosition() - m_elevatorProperties.kOffsetDistance.in(Meters);
        return Distance.ofRelativeUnits(MathUtil.angleModulus(_distance), Meters);
    }

    @Override
    public void setMotorBrake(boolean enable) {
        var config = new SparkMaxConfig();

        config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        tryUntilOk(
                m_primaryMotor,
                5,
                () -> m_primaryMotor.configure(
                        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
        if (m_hasfollowMotor) {
            tryUntilOk(
                    m_followerMotor,
                    5,
                    () -> m_followerMotor.configure(
                            config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
        }
    }
}
