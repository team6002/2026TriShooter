package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.SparkUtil.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

public class PivotIOSpark implements PivotIO {
    PivotProperties m_pivotProperties;
    public boolean configurationFailed = false;
    private boolean m_hasFollowerMotor = false;
    // private Angle m_setpoint;
    private Angle m_minAngle, m_maxAngle;

    private TrapezoidProfile m_profile;
    private Timer m_timer;
    private TrapezoidProfile.State m_setpoint;
    private TrapezoidProfile.State m_goal;
    private double m_feedforward;
    private double m_manualValue;

    public static TrapezoidProfile.Constraints kArmMotionConstraint;
    public final ArmFeedforward kArmFeedforward;
    // Hardware objects
    private final SparkMax m_primaryMotor;
    private final SparkMax m_followerMotor;

    private final SparkAbsoluteEncoder m_absoluteEncoder;
    private final SparkClosedLoopController m_motorController;
    // Connection debouncers
    private final Debouncer primaryConnectedDebounce = new Debouncer(0.5);
    private final Debouncer absoluteEncoderConnectedDebounce = new Debouncer(0.5);
    private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

    AngularVelocity m_maxAngularVelocity;
    AngularAcceleration m_maxAnglarAcceleration;

    public PivotIOSpark(PivotProperties pivotProperties) {
        m_pivotProperties = pivotProperties;
        kArmFeedforward = pivotProperties.getArmFeedforward();
        m_minAngle = pivotProperties.kminAngle;
        m_maxAngle = pivotProperties.kmaxAngle;
        m_maxAngularVelocity = pivotProperties.kMaxVelocity;
        m_maxAnglarAcceleration = pivotProperties.kMaxAcceleration;
        kArmMotionConstraint = new TrapezoidProfile.Constraints(
                m_maxAngularVelocity.in(RadiansPerSecond), m_maxAnglarAcceleration.in(RadiansPerSecondPerSecond));
        m_profile = new TrapezoidProfile(kArmMotionConstraint);
        // Hardware Creation
        m_primaryMotor = new SparkMax(pivotProperties.primaryCanID, MotorType.kBrushless);
        m_absoluteEncoder = m_primaryMotor.getAbsoluteEncoder();
        m_motorController = m_primaryMotor.getClosedLoopController();

        // Configure Motor
        sparkStickyFault = false;
        tryUntilOk(
                m_primaryMotor,
                5,
                () -> m_primaryMotor.configure(
                        pivotProperties.getPrimaryConfig(),
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters));

        if (pivotProperties.hasFollowerMotor) m_hasFollowerMotor = true;
        if (m_hasFollowerMotor) {
            m_followerMotor = new SparkMax(pivotProperties.followerCanID, MotorType.kBrushless);
            tryUntilOk(
                    m_followerMotor,
                    5,
                    () -> m_followerMotor.configure(
                            pivotProperties.getFollowerConfig(),
                            ResetMode.kResetSafeParameters,
                            PersistMode.kPersistParameters));
        } else {
            m_followerMotor = null;
        }
        configurationFailed = sparkStickyFault;

        m_timer = new Timer();
        m_timer.start();

        m_setpoint = new TrapezoidProfile.State(getArmAngle().in(Radians), 0);
        m_goal = m_setpoint;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.configurationFailed = this.configurationFailed;
        inputs.hasFollowerMotor = this.m_hasFollowerMotor;
        inputs.setPoint = Angle.ofBaseUnits(m_goal.position, Radians);
        inputs.manualValue = m_manualValue;
        // Obtain encoder Readings
        inputs.currentAngle = getArmAngle();
        sparkStickyFault = false;
        ifOk(m_primaryMotor, m_absoluteEncoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        inputs.absoluteEncoderConnected = absoluteEncoderConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(
                m_primaryMotor,
                new DoubleSupplier[] {m_primaryMotor::getAppliedOutput, m_primaryMotor::getBusVoltage},
                (values) -> inputs.primaryMotorOutputVolts = values[0] * values[1]);
        ifOk(m_primaryMotor, m_primaryMotor::getOutputCurrent, (value) -> inputs.primaryMotorSupplyCurrentAmps = value);
        inputs.primaryMotorConnected = primaryConnectedDebounce.calculate(!sparkStickyFault);

        if (m_hasFollowerMotor) {
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
    public void setIdle() {
        m_setpoint = new TrapezoidProfile.State(getArmAngle().in(Radians), 0);
        m_goal = m_setpoint;
    }

    @Override
    public void runAutomatic() {
        m_setpoint = m_profile.calculate(0.02, m_setpoint, m_goal);
        double _position = m_setpoint.position + m_pivotProperties.kArmZeroCosineOffset.in(Radians);
        m_feedforward = kArmFeedforward.calculate(getArmAngle().in(Radians), m_goal.velocity);
        m_motorController.setReference(_position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward);
    }

    @Override
    public void runManual(double _power) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
        // passively
        setIdle();
        // update the feedforward variable with the newly zero target velocity
        m_feedforward = kArmFeedforward.calculate(getArmAngle().in(Radians), m_goal.velocity);
        // set the power of the motor
        m_primaryMotor.set(_power + (m_feedforward / 12.0));
        m_manualValue = _power; // this variable is only used for logging or debugging if needed
    }

    @Override
    public void setTargetPosition(Angle _setpoint) {
        // _setpoint = Angle.ofBaseUnits(
        //         MathUtil.clamp(_setpoint.in(Radians), m_minAngle.in(Radians), m_maxAngle.in(Radians)), Radians);

        _setpoint.plus(m_pivotProperties.kArmZeroCosineOffset);
        if (_setpoint.in(Radians) != m_goal.position) {
            m_setpoint = new TrapezoidProfile.State(getArmAngle().in(Radians), 0);
            m_goal = new TrapezoidProfile.State(_setpoint.in(Radians), 0);
        }
    }

    // private void updateMotionProfile() {
    //     m_startState = new TrapezoidProfile.State(getArmAngle().in(Radians), m_absoluteEncoder.getVelocity());
    //     m_endState = new TrapezoidProfile.State(m_setpoint.in(Radian), 0.0);
    //     m_profile = new TrapezoidProfile(kArmMotionConstraint);
    //     m_timer.reset();
    // }

    public Angle getArmAngle() {
        double _angle = m_absoluteEncoder.getPosition() - m_pivotProperties.kArmZeroCosineOffset.in(Radians);
        return Angle.ofRelativeUnits(_angle, Radians);
    }

    public AngularVelocity getAngularVelocity() {
        return AngularVelocity.ofBaseUnits(m_absoluteEncoder.getVelocity(), RadiansPerSecond);
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
        if (m_hasFollowerMotor) {
            tryUntilOk(
                    m_followerMotor,
                    5,
                    () -> m_followerMotor.configure(
                            config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
        }
    }

    @Override
    public AngularVelocity getMaxAngularVelocity() {
        return m_maxAngularVelocity;
    }

    @Override
    public AngularAcceleration getMaxAngularAcceleration() {
        return m_maxAnglarAcceleration;
    }
}
