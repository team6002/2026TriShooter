package frc.robot.subsystems.conveyor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ConveyorIOSpark implements ConveyorIO {
    private final SparkMax conveyorMotor;
    private final SparkMax conveyorFollowerMotor;
    private final RelativeEncoder conveyorEncoder;
    private final SparkClosedLoopController conveyorController;

    private double conveyorReference;
    private ControlType conveyorType;

    public ConveyorIOSpark() {
        // initialize motor
        conveyorMotor = new SparkMax(ConveyorConstants.kConveyorCanId, MotorType.kBrushless);
        conveyorFollowerMotor =
                new SparkMax(ConveyorConstants.kConveyorFollowerCanId, MotorType.kBrushless);

        // initialize PID controller
        conveyorController = conveyorMotor.getClosedLoopController();

        // initalize encoder
        conveyorEncoder = conveyorMotor.getEncoder();

        // apply config
        conveyorMotor.configure(
                ConveyorConfig.conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        conveyorFollowerMotor.configure(
                ConveyorConfig.conveyorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        conveyorReference = 0;
        conveyorType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(ConveyorIOInputs inputs) {
        inputs.conveyorReference = getReference();
        inputs.conveyorCurrent = getCurrent();
        inputs.conveyorVoltage = getVoltage();
        inputs.conveyorVelocity = getVelocity();
    }

    @Override
    public double getVelocity() {
        return conveyorEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return conveyorMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return conveyorMotor.getBusVoltage() * conveyorMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return conveyorReference;
    }

    @Override
    public void setVoltage(double voltage) {
        conveyorReference = voltage;
        conveyorType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double velocity) {
        conveyorReference = velocity;
        conveyorType = ControlType.kVelocity;
    }

    @Override
    public void PID() {
        conveyorController.setSetpoint(conveyorReference, conveyorType);
    }
}
