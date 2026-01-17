package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimbIOSpark implements ClimbIO {
    private final SparkMax shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final SparkClosedLoopController shooterController;

    private double shooterReference;
    private ControlType shooterType;

    public ClimbIOSpark() {
        // initialize motor
        shooterMotor = new SparkMax(ClimbConstants.kClimbCanId, MotorType.kBrushless);

        // initialize PID controller
        shooterController = shooterMotor.getClosedLoopController();

        // initalize encoder
        shooterEncoder = shooterMotor.getEncoder();

        // apply config
        shooterMotor.configure(
                ClimbConfig.climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        shooterReference = 0;
        shooterType = ControlType.kPosition;
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbReference = getReference();
        inputs.climbCurrent = getCurrent();
        inputs.climbVoltage = getVoltage();
        inputs.climbVelocity = getVelocity();
    }

    @Override
    public double getVelocity() {
        return shooterEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return shooterMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return shooterMotor.getBusVoltage() * shooterMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return shooterReference;
    }

    @Override
    public void setVoltage(double voltage) {
        shooterReference = voltage;
        shooterType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double velocity) {
        shooterReference = velocity;
        shooterType = ControlType.kVelocity;
    }

    @Override
    public void PID() {
        shooterController.setSetpoint(shooterReference, shooterType);
    }
}
