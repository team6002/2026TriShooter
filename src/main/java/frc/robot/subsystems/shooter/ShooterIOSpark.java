package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;

public class ShooterIOSpark implements ShooterIO {
    private final SparkMax middleShooterMotor, leftShooterMotor, rightShooterMotor;

    private final RelativeEncoder middleShooterEncoder, leftShooterEncoder, rightShooterEncoder;

    private double leftShooterReference, middleShooterReference, rightShooterReference;

    //tuning
    // private final LoggedNetworkNumber leftS, leftV, middleS, middleV, rightS, rightV, reference;

    public ShooterIOSpark() {
        // initialize motor
        middleShooterMotor = new SparkMax(ShooterConstants.kShooterCanId, MotorType.kBrushless);
        leftShooterMotor = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

        // initalize encoder
        middleShooterEncoder = middleShooterMotor.getEncoder();
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        // apply config
        middleShooterMotor.configure(
                ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftShooterMotor.configure(
                ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightShooterMotor.configure(
            ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //tuning

        // leftS = new LoggedNetworkNumber("Shooter/leftS", 0.0);
        // leftV = new LoggedNetworkNumber("Shooter/leftV", 0.0);
        // middleS = new LoggedNetworkNumber("Shooter/middleS", 0.0);
        // middleV = new LoggedNetworkNumber("Shooter/middleV", 0.0);
        // rightS = new LoggedNetworkNumber("Shooter/rightS", 0.0);
        // rightV = new LoggedNetworkNumber("Shooter/rightV", 0.0);
        // reference = new LoggedNetworkNumber("Shooter/reference", 0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterReference = Units.radiansToDegrees(getReference());

        inputs.shooterCurrent = getCurrent();
        inputs.shooterVoltage = getVoltage();
        inputs.shooterVelocity = Units.radiansToDegrees(getVelocity());

        inputs.leftShooterCurrent = getLeftCurrent();
        inputs.leftShooterVoltage = getLeftVoltage();
        inputs.leftShooterVelocity = Units.radiansToDegrees(getLeftVelocity());

        inputs.rightShooterCurrent = getRightCurrent();
        inputs.rightShooterVoltage = getRightVoltage();
        inputs.rightShooterVelocity = Units.radiansToDegrees(getRightVelocity());
    }

    @Override
    public double getVelocity() {
        return middleShooterEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return middleShooterMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return middleShooterMotor.getBusVoltage() * middleShooterMotor.getAppliedOutput();
    }

    @Override
    public double getLeftVelocity() {
        return leftShooterEncoder.getVelocity();
    }

    @Override
    public double getLeftCurrent(){
        return leftShooterMotor.getOutputCurrent();
    }

    @Override
    public double getLeftVoltage() {
        return leftShooterMotor.getBusVoltage() * leftShooterMotor.getAppliedOutput();
    }

    @Override
    public double getRightVelocity(){
        return rightShooterEncoder.getVelocity();
    }

    @Override
    public double getRightCurrent(){
        return rightShooterMotor.getOutputCurrent();
    }

    @Override
    public double getRightVoltage() {
        return rightShooterMotor.getBusVoltage() * rightShooterMotor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double voltage) {
        middleShooterReference = voltage;
    }

    @Override
    public void setLeftVoltage(double voltage) {
        leftShooterReference = voltage;
    }

    @Override
    public void setRightVoltage(double voltage) {
        rightShooterReference = voltage;
    }

    @Override
    public void periodic(){
        leftShooterMotor.setVoltage(leftShooterReference);
        middleShooterMotor.setVoltage(middleShooterReference);
        rightShooterMotor.setVoltage(rightShooterReference);

        //tuning

        // leftShooterMotor.setVoltage(leftS.get() + (leftV.get() * Units.degreesToRadians(reference.get())));
        // middleShooterMotor.setVoltage(middleS.get() + (middleV.get() * Units.degreesToRadians(reference.get())));
        // rightShooterMotor.setVoltage(rightS.get() + (rightV.get() * Units.degreesToRadians(reference.get())));
    }
}
