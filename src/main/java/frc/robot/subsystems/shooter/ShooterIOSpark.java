package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class ShooterIOSpark implements ShooterIO {
    private final SparkMax shooterMotor;
    private final SparkMax leftShooterMotor;
    private final SparkMax rightShooterMotor;

    private final RelativeEncoder shooterEncoder;
    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;


    private final SparkClosedLoopController shooterController;
    private final SparkClosedLoopController leftShooterController;
    private final SparkClosedLoopController rightShooterController;


    private double shooterReference;
    private ControlType shooterType;

    LoggedNetworkNumber shooterP = new LoggedNetworkNumber("/Tuning/ShooterP", 0.001);
    LoggedNetworkNumber shooterI = new LoggedNetworkNumber("/Tuning/ShooterI", 0.0);
    LoggedNetworkNumber shooterD = new LoggedNetworkNumber("/Tuning/ShooterD", 0.0);
    LoggedNetworkNumber shooterV = new LoggedNetworkNumber("/Tuning/ShooterV", 0.02);
    LoggedNetworkNumber shooterS = new LoggedNetworkNumber("/Tuning/ShooterS", 0.16);
    LoggedNetworkNumber shooterA = new LoggedNetworkNumber("/Tuning/ShooterA", 0.0);
    LoggedNetworkNumber shooterTargetVel = new LoggedNetworkNumber("/Tuning/ShooterTargetVel", 0.0);

    // Cache last-applied values to detect changes
    private double lastP = Double.NaN;
    private double lastI = Double.NaN;
    private double lastD = Double.NaN;

    private double lastS = Double.NaN;
    private double lastV = Double.NaN;
    private double lastA = Double.NaN;

    private double lastTargetDeg = Double.NaN;



    public ShooterIOSpark() {
        // initialize motor
        shooterMotor = new SparkMax(ShooterConstants.kShooterCanId, MotorType.kBrushless);
        leftShooterMotor = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

        // initialize PID controller
        shooterController = shooterMotor.getClosedLoopController();
        leftShooterController = leftShooterMotor.getClosedLoopController();
        rightShooterController = rightShooterMotor.getClosedLoopController();

        // initalize encoder
        shooterEncoder = shooterMotor.getEncoder();
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        // apply config
        shooterMotor.configure(
                ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftShooterMotor.configure(
                ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightShooterMotor.configure(
            ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // reset target speed in init
        shooterReference = 0;
        shooterType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterReference = getReference();
        inputs.shooterCurrent = getCurrent();
        inputs.shooterVoltage = getVoltage();
        inputs.shooterVelocity = getVelocity();
        inputs.shooterPos = shooterEncoder.getPosition();

        inputs.leftShooterCurrent = leftShooterMotor.getOutputCurrent();
        inputs.leftShooterVoltage = leftShooterMotor.getAppliedOutput() * leftShooterMotor.getBusVoltage();
        inputs.leftShooterVelocity = leftShooterEncoder.getVelocity();

        inputs.rightShooterCurrent = rightShooterMotor.getOutputCurrent();
        inputs.rightShooterVoltage = rightShooterMotor.getAppliedOutput() * rightShooterMotor.getBusVoltage();
        inputs.rightShooterVelocity = rightShooterEncoder.getVelocity();
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
        shooterType = ControlType.kMAXMotionVelocityControl;
    }

    @Override
    public void PID() {
        double p = shooterP.get();
        double i = shooterI.get();
        double d = shooterD.get();

        double s = shooterS.get();
        double v = shooterV.get();
        double a = shooterA.get();

        double targetDeg = shooterTargetVel.get();

        boolean pidChanged = (p != lastP) || (i != lastI) || (d != lastD);
        boolean ffChanged = (s != lastS) || (v != lastV) || (a != lastA);

        // Only reconfigure SparkMax if PID or FF changed
        if (pidChanged || ffChanged) {
            SparkBaseConfig newConfig = new SparkMaxConfig();
            newConfig.apply(ShooterConfig.shooterConfig);

            if (pidChanged) {
                newConfig.closedLoop.pid(p, i, d);
                lastP = p;
                lastI = i;
                lastD = d;
            }

            if (ffChanged) {
                newConfig.closedLoop.feedForward.sva(s, v, a);
                lastS = s;
                lastV = v;
                lastA = a;
            }

            shooterMotor.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );

            rightShooterMotor.configure(
                newConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );
        }

        // Only update reference if target velocity changed
        if (targetDeg != lastTargetDeg) {
            shooterType = ControlType.kVelocity;
            setReference(Math.toRadians(targetDeg));
            lastTargetDeg = targetDeg;
        }

        // Command the motor
        shooterController.setSetpoint(shooterReference, shooterType);
        rightShooterController.setSetpoint(shooterReference, shooterType);
    }
}
