package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;

public class HoodIOSpark implements HoodIO {
    private final SparkMax hoodMotor;
    private final RelativeEncoder hoodEncoder;
    private final SparkClosedLoopController hoodController;

    private double hoodReference;
    private ControlType hoodType;

    public HoodIOSpark() {
        // initialize motor
        hoodMotor = new SparkMax(HoodConstants.kHoodCanId, MotorType.kBrushless);

        // initialize PID controller
        hoodController = hoodMotor.getClosedLoopController();

        // initalize encoder
        hoodEncoder = hoodMotor.getEncoder();

        // apply config
        hoodMotor.configure(
                HoodConfig.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        hoodReference = 0;
        hoodType = ControlType.kPosition;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodReference = getReference();
        inputs.hoodCurrent = getCurrent();
        inputs.hoodVoltage = getVoltage();
        inputs.hoodVelocity = getVelocity();
        inputs.hoodPos = Units.radiansToDegrees(getPosition());
    }

    @Override
    public double getVelocity() {
        return hoodEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return hoodMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return hoodMotor.getBusVoltage() * hoodMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return hoodReference;
    }

    @Override
    public void setVoltage(double voltage) {
        hoodReference = voltage;
        hoodType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double angRad) {
        hoodReference = angRad;
        hoodType = ControlType.kPosition;
    }

    @Override
    public boolean atReference(){
        return Math.abs(getReference() - getPosition()) < HoodConstants.kTolerance;
    }

    @Override
    public void PID() {
        hoodController.setSetpoint(hoodReference, hoodType);
    }
}
