package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMax intakeMotorFollower;
    private final RelativeEncoder intakeEncoder;
    private final SparkClosedLoopController intakeController;

    private double intakeReference;
    private ControlType intakeType;

    public IntakeIOSpark() {
        // initialize motor
        intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
        intakeMotorFollower = new SparkMax(IntakeConstants.kIntakeFollowerCanId, MotorType.kBrushless);

        // initialize PID controller
        intakeController = intakeMotor.getClosedLoopController();

        // initalize encoder
        intakeEncoder = intakeMotor.getEncoder();

        // apply config
        intakeMotor.configure(
                IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
        intakeMotorFollower.configure(
                IntakeConfig.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        intakeReference = 0;
        intakeType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeReference = getReference();
        inputs.intakeCurrent = getCurrent();
        inputs.intakeVoltage = getVoltage();
        inputs.intakeVelocity = getVelocity();
    }

    @Override
    public double getVelocity() {
        return intakeEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return intakeReference;
    }

    @Override
    public void setVoltage(double voltage) {
        intakeReference = voltage;
        intakeType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double velocity) {
        intakeReference = velocity;
        intakeType = ControlType.kVelocity;
    }

    @Override
    public void PID() {
        intakeController.setSetpoint(intakeReference, intakeType);
    }
}
