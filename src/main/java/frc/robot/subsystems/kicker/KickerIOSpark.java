package frc.robot.subsystems.kicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class KickerIOSpark implements KickerIO {
    private final SparkMax kickerMotor;
    private final RelativeEncoder kickerEncoder;
    private final SparkClosedLoopController kickerController;

    private double kickerReference;
    private ControlType kickerType;

    public KickerIOSpark() {
        // initialize motor
        kickerMotor = new SparkMax(KickerConstants.kKickerCanId, MotorType.kBrushless);

        // initialize PID controller
        kickerController = kickerMotor.getClosedLoopController();

        // initalize encoder
        kickerEncoder = kickerMotor.getEncoder();

        // apply config
        kickerMotor.configure(
                KickerConfig.kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        kickerReference = 0;
        kickerType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.kickerReference = getReference();
        inputs.kickerCurrent = getCurrent();
        inputs.kickerVoltage = getVoltage();
        inputs.kickerVelocity = getVelocity();
    }

    @Override
    public double getVelocity() {
        return kickerEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return kickerMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return kickerMotor.getBusVoltage() * kickerMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return kickerReference;
    }

    @Override
    public void setVoltage(double voltage) {
        kickerReference = voltage;
        kickerType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double velocity) {
        kickerReference = velocity;
        kickerType = ControlType.kVelocity;
    }

    @Override
    public void PID() {
        kickerController.setSetpoint(kickerReference, kickerType);
    }
}
