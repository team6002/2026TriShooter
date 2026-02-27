package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class IntakeIOSpark implements IntakeIO {
    private final SparkFlex intakeMotor;
    private final SparkFlex intakeFollowerMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkClosedLoopController intakeController;

    private double intakeReference;
    private ControlType intakeType;

    private final SparkMax intakeExtenderMotor;
    private final AbsoluteEncoder intakeExtenderEncoder;
    private final SparkClosedLoopController intakeExtenderController;

    private double intakeExtenderReference;
    private ControlType intakeExtenderType;

    public IntakeIOSpark() {
        // initialize motor
        intakeMotor = new SparkFlex(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
        intakeFollowerMotor = new SparkFlex(IntakeConstants.kIntakeFollowerCanId, MotorType.kBrushless);

        intakeExtenderMotor = new SparkMax(ExtenderConstants.kIntakeExtenderCanId, MotorType.kBrushless);

        // initialize PID controller
        intakeController = intakeMotor.getClosedLoopController();
        intakeExtenderController = intakeExtenderMotor.getClosedLoopController();

        // initalize encoder
        intakeEncoder = intakeMotor.getEncoder();
        intakeExtenderEncoder = intakeExtenderMotor.getAbsoluteEncoder();

        // apply config
        intakeMotor.configure(
                IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeFollowerMotor.configure(
                IntakeConfig.intakeFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeExtenderMotor.configure(
                IntakeConfig.intakeExtenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        intakeReference = 0;
        intakeType = ControlType.kVelocity;

        intakeExtenderReference = ExtenderConstants.kStow;
        intakeExtenderType = ControlType.kMAXMotionPositionControl;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeReference = getReference();
        inputs.intakeCurrent = getCurrent();
        inputs.intakeVoltage = getVoltage();
        inputs.intakeVelocity = getVelocity();
        inputs.intakePosition = getPosition();
        inputs.intakeTemp = Fahrenheit.convertFrom(intakeMotor.getMotorTemperature(), Celsius);

        inputs.intakeFollowerTemp = Fahrenheit.convertFrom(intakeFollowerMotor.getMotorTemperature(), Celsius);

        inputs.extenderReference = Units.radiansToDegrees(getExtenderReference());
        inputs.extenderCurrent = getExtenderCurrent();
        inputs.extenderVoltage = getExtenderVoltage();
        inputs.extenderVelocity = getExtenderVelocity();
        inputs.extenderPosition = Units.radiansToDegrees(getExtenderPosition());
        inputs.extenderInPosition = getExtenderInPosition();
        inputs.extenderTemp = Fahrenheit.convertFrom(intakeExtenderMotor.getMotorTemperature(), Celsius);
    }

    @Override
    public void setReference(double velocity) {
        intakeReference = velocity;
        intakeType = ControlType.kVelocity;
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
    public double getVoltage() {
        return intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
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
    public double getPosition() {
        return intakeEncoder.getPosition();
    }

    @Override
    public void setExtenderReference(double angRad) {
        intakeExtenderReference = angRad;
        intakeExtenderType = ControlType.kMAXMotionPositionControl;
    }

    @Override
    public double getExtenderReference() {
        return intakeExtenderReference;
    }

    @Override
    public void setExtenderVoltage(double voltage) {
        intakeExtenderReference = voltage;
        intakeExtenderType = ControlType.kVoltage;
    }

    @Override
    public double getExtenderVoltage() {
        return intakeExtenderMotor.getBusVoltage() * intakeExtenderMotor.getAppliedOutput();
    }

    @Override
    public double getExtenderVelocity() {
        return intakeExtenderEncoder.getVelocity();
    }

    @Override
    public double getExtenderCurrent() {
        return intakeExtenderMotor.getOutputCurrent();
    }

    @Override
    public double getExtenderPosition() {
        return intakeExtenderEncoder.getPosition();
    }

    @Override
    public boolean getExtenderInPosition() {
        double positionError = Math.abs(intakeExtenderEncoder.getPosition() - intakeExtenderReference);
        return positionError < ExtenderConstants.kPositionTolerance;
    }

    @Override
    public void setExtenderLowCurrentMode(boolean lowCurrentMode) {
        if (lowCurrentMode) {
            SparkFlexConfig newLeadConfig = new SparkFlexConfig();
            newLeadConfig.apply(IntakeConfig.intakeConfig);
            newLeadConfig.smartCurrentLimit(5);

            SparkFlexConfig newFollowerConfig = new SparkFlexConfig();
            newFollowerConfig.apply(IntakeConfig.intakeFollowerConfig);
            newFollowerConfig.smartCurrentLimit(5);

            intakeMotor.configureAsync(
                    newLeadConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            intakeFollowerMotor.configureAsync(
                    newFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        } else {
            SparkFlexConfig newLeadConfig = new SparkFlexConfig();
            newLeadConfig.apply(IntakeConfig.intakeConfig);
            newLeadConfig.smartCurrentLimit(40);

            SparkFlexConfig newFollowerConfig = new SparkFlexConfig();
            newFollowerConfig.apply(IntakeConfig.intakeFollowerConfig);
            newFollowerConfig.smartCurrentLimit(40);

            intakeMotor.configureAsync(
                    newLeadConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            intakeFollowerMotor.configureAsync(
                    newFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void periodic() {
        intakeController.setSetpoint(intakeReference, intakeType);

        // horizontal is 270, offset to 0

        double ff = -ExtenderConstants.kG * (Math.cos(getExtenderPosition() - Math.toRadians(270)));
        intakeExtenderController.setSetpoint(intakeExtenderReference, intakeExtenderType, ClosedLoopSlot.kSlot0, ff);
    }
}
