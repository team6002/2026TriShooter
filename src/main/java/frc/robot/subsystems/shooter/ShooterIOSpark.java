package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterIOSpark implements ShooterIO {
    
    public enum ControlStage {
        SPIN_UP, HOLD_WHEN_READY, HOLD
    }
    
    private final SparkMax shooterMotor, leftShooterMotor, rightShooterMotor;
    private final RelativeEncoder shooterEncoder, leftShooterEncoder, rightShooterEncoder;
    private final SparkClosedLoopController shooterController, leftShooterController, rightShooterController;

    private boolean shootTimerEnabled;
    private double shootingTimeStarted;
    private double voltTimeFactor = 0.2; // add 2 volts / 10 seconds
    
    private ControlStage controlStage = ControlStage.SPIN_UP;
    private double shooterReference = 0;
    private double feedforwardSlot = 0;
    private ControlType shooterType = ControlType.kVelocity;
    
    private boolean onTarget = false;
    private CircularBuffer voltageBuffer = new CircularBuffer(ShooterConstants.kMinOnTargetSamples + 5);
    
    private double[][] motorFeedforwardCoeffs = new double[3][4];
    private double shooterKs, shooterKv, leftKs, leftKv, rightKs, rightKv;
    
    private final LoggedNetworkNumber p = new LoggedNetworkNumber("/Tuning/Shooter/kP", 0.001);
    private final LoggedNetworkNumber d = new LoggedNetworkNumber("/Tuning/Shooter/kD", 0.0);
    private final LoggedNetworkNumber leftP = new LoggedNetworkNumber("/Tuning/Shooter/kLeftP", 0.001);
    private final LoggedNetworkNumber leftD = new LoggedNetworkNumber("/Tuning/Shooter/kLeftD", 0.0);
    private final LoggedNetworkNumber rightP = new LoggedNetworkNumber("/Tuning/Shooter/kRightP", 0.001);
    private final LoggedNetworkNumber rightD = new LoggedNetworkNumber("/Tuning/Shooter/kRightD", 0.0);
    private final LoggedNetworkNumber reference = new LoggedNetworkNumber("/Tuning/Shooter/TargetVel", 0.0);
    private final LoggedNetworkNumber startOnTargetRpm = new LoggedNetworkNumber("/Tuning/Shooter/StartOnTargetRpm", ShooterConstants.kStartOnTargetVel);
    private final LoggedNetworkNumber stopOnTargetRpm = new LoggedNetworkNumber("/Tuning/Shooter/StopOnTargetRpm", ShooterConstants.kStopOnTargetVel);
    
    private double lastP = Double.NaN, lastD = Double.NaN, lastLeftP = Double.NaN, lastLeftD = Double.NaN;
    private double lastRightP = Double.NaN, lastRightD = Double.NaN, lastTargetVel = Double.NaN, lastFeedforwardSlot = Double.NaN;

    public ShooterIOSpark() {
        shooterMotor = new SparkMax(ShooterConstants.kShooterCanId, MotorType.kBrushless);
        leftShooterMotor = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
        rightShooterMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

        shooterController = shooterMotor.getClosedLoopController();
        leftShooterController = leftShooterMotor.getClosedLoopController();
        rightShooterController = rightShooterMotor.getClosedLoopController();

        shooterEncoder = shooterMotor.getEncoder();
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        shooterMotor.configure(ShooterConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooterMotor.configure(ShooterConfig.leftShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooterMotor.configure(ShooterConfig.rightShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializeFeedforwardCoefficients();
    }

    private void initializeFeedforwardCoefficients() {
        motorFeedforwardCoeffs[0][0] = ShooterConstants.kFreePID[0][3];
        motorFeedforwardCoeffs[0][1] = ShooterConstants.kFreePID[0][4];
        motorFeedforwardCoeffs[1][0] = ShooterConstants.kFreePID[1][3];
        motorFeedforwardCoeffs[1][1] = ShooterConstants.kFreePID[1][4];
        motorFeedforwardCoeffs[2][0] = ShooterConstants.kFreePID[2][3];
        motorFeedforwardCoeffs[2][1] = ShooterConstants.kFreePID[2][4];
        motorFeedforwardCoeffs[0][2] = ShooterConstants.kLoadPID[0][3];
        motorFeedforwardCoeffs[0][3] = ShooterConstants.kLoadPID[0][4];
        motorFeedforwardCoeffs[1][2] = ShooterConstants.kLoadPID[1][3];
        motorFeedforwardCoeffs[1][3] = ShooterConstants.kLoadPID[1][4];
        motorFeedforwardCoeffs[2][2] = ShooterConstants.kLoadPID[2][3];
        motorFeedforwardCoeffs[2][3] = ShooterConstants.kLoadPID[2][4];
        
        updateFeedforwardCoefficients(0);
    }

    private void updateFeedforwardCoefficients(int slot) {
        int freeSlot = (slot == 0) ? 0 : 2;
        int kvOffset = (slot == 0) ? 1 : 3;
        shooterKs = motorFeedforwardCoeffs[1][freeSlot];
        shooterKv = motorFeedforwardCoeffs[1][kvOffset];
        leftKs = motorFeedforwardCoeffs[0][freeSlot];
        leftKv = motorFeedforwardCoeffs[0][kvOffset];
        rightKs = motorFeedforwardCoeffs[2][freeSlot];
        rightKv = motorFeedforwardCoeffs[2][kvOffset];
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterReference = Units.radiansToDegrees(shooterReference);
        inputs.shooterCurrent = getCurrent();
        inputs.shooterVoltage = getVoltage();
        inputs.shooterVelocity = Units.radiansToDegrees(getVelocity());
        inputs.shooterPos = shooterEncoder.getPosition();
        inputs.leftShooterCurrent = leftShooterMotor.getOutputCurrent();
        inputs.leftShooterVoltage = leftShooterMotor.getAppliedOutput() * leftShooterMotor.getBusVoltage();
        inputs.leftShooterVelocity = Units.radiansToDegrees(leftShooterEncoder.getVelocity());
        inputs.rightShooterCurrent = rightShooterMotor.getOutputCurrent();
        inputs.rightShooterVoltage = rightShooterMotor.getAppliedOutput() * rightShooterMotor.getBusVoltage();
        inputs.rightShooterVelocity = Units.radiansToDegrees(rightShooterEncoder.getVelocity());
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
    public void startShootingTimer(){
        shootTimerEnabled = true;
        shootingTimeStarted = Timer.getFPGATimestamp();
    }

    @Override
    public void stopShootingTimer(){
        shootTimerEnabled = false;
    }

    private double getElapsedTime(){
        return Timer.getFPGATimestamp() - shootingTimeStarted;
    }

    public ControlStage getControlStage() { 
        return controlStage; 
    }

    public boolean isOnTarget() { 
        return controlStage == ControlStage.HOLD; 
    }

    @Override
    public void setVoltage(double voltage) {
        shooterReference = voltage;
        shooterType = ControlType.kVoltage;
        controlStage = ControlStage.SPIN_UP;
        resetHoldState();
    }

    @Override
    public void setReference(double velocity) {
        if (shooterReference != velocity || shooterType != ControlType.kVelocity) {
            shooterReference = velocity;
            shooterType = ControlType.kVelocity;
            controlStage = ControlStage.SPIN_UP;
            resetHoldState();
        }
    }

    @Override
    public void setFeedforwardSlot(double slot) {
        feedforwardSlot = slot;
    }

    @Override
    public void setControlStage(ControlStage stage) {
        if (stage != controlStage) {
            controlStage = stage;
            if (stage == ControlStage.HOLD) {
                transitionToHold();
            } else {
                resetHoldState();
            }
        }
    }

    @Override
    public void PID() {
        Logger.recordOutput("Shooter/ControlStage", getControlStage().toString());
        if (feedforwardSlot != lastFeedforwardSlot) {
            updateFeedforwardCoefficients((int)feedforwardSlot);
            lastFeedforwardSlot = feedforwardSlot;
        }
        updateGains();
        updateControl();
    }

    private void updateGains() {
        SparkMaxConfig newMiddleConfig = new SparkMaxConfig();
        newMiddleConfig.apply(ShooterConfig.shooterConfig);
        SparkMaxConfig newLeftConfig = new SparkMaxConfig();
        newLeftConfig.apply(ShooterConfig.leftShooterConfig);
        SparkMaxConfig newRightConfig = new SparkMaxConfig();
        newRightConfig.apply(ShooterConfig.rightShooterConfig);

        if(!(lastP == p.get()) || !(lastD == d.get())){
            newMiddleConfig.closedLoop.pid(p.get(), 0, d.get(), ClosedLoopSlot.kSlot0);
            shooterMotor.configure(newMiddleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            lastP = p.get();
            lastD = d.get();
        }

        if(!(lastLeftP == leftP.get()) || !(lastLeftD == leftD.get())){
            newLeftConfig.closedLoop.pid(leftP.get(), 0, leftD.get(), ClosedLoopSlot.kSlot0);
            leftShooterMotor.configure(newLeftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            lastLeftP = leftP.get();
            lastLeftD = leftD.get();
        }

        if(!(lastRightP == rightP.get()) || !(lastRightD == rightD.get())){
            newRightConfig.closedLoop.pid(rightP.get(), 0, rightD.get(), ClosedLoopSlot.kSlot0);
            rightShooterMotor.configure(newRightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            lastRightP = rightP.get();
            lastRightD = rightD.get();
        }

        if(!(lastTargetVel == reference.get())){
            setReference(Math.toRadians(reference.get()));
            lastTargetVel = Math.toRadians(reference.get());
        }
    }

    private void updateControl() {
        double currentRpm = getVelocity();
        double appliedVoltage = getVoltage();
        
        switch(controlStage) {
            case SPIN_UP: {
                double ffMiddle = shooterKs + shooterKv * shooterReference;
                double ffLeft   = leftKs   + leftKv   * shooterReference;
                double ffRight  = rightKs  + rightKv  * shooterReference;

                ffMiddle = Math.min(ffMiddle, ShooterConstants.kMaxFeedforwardVoltage);
                ffLeft   = Math.min(ffLeft,   ShooterConstants.kMaxFeedforwardVoltage);
                ffRight  = Math.min(ffRight,  ShooterConstants.kMaxFeedforwardVoltage);

                shooterController.setSetpoint(
                    shooterReference,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    ffMiddle
                );

                leftShooterController.setSetpoint(
                    shooterReference,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    ffLeft
                );

                rightShooterController.setSetpoint(
                    shooterReference,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    ffRight
                );

                resetHoldState();
                break;
            }
            case HOLD_WHEN_READY:
                double error = Math.abs(currentRpm - shooterReference);
                boolean nowOnTarget = onTarget 
                    ? error < stopOnTargetRpm.get()
                    : error < startOnTargetRpm.get();
                
                if (nowOnTarget && !onTarget) {
                    onTarget = true;
                } else if (!nowOnTarget) {
                    resetHoldState();
                }
                
                if (onTarget) {
                    voltageBuffer.addValue(appliedVoltage);
                }
                
                leftShooterController.setSetpoint(shooterReference, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
                
                if (voltageBuffer.getNumValues() >= ShooterConstants.kMinOnTargetSamples) {
                    transitionToHold();
                }
                break;
            case HOLD:
                double shooterTimeVolts = 0;
                if(shootTimerEnabled) {
                    shooterTimeVolts = getElapsedTime() * voltTimeFactor;
                }

                double ffVoltage = shooterKs + (shooterKv * shooterReference) + shooterTimeVolts;
                double leftFfVoltage = leftKs + (leftKv * shooterReference) + shooterTimeVolts;
                double rightFfVoltage = rightKs + (rightKv * shooterReference) + shooterTimeVolts;

                Logger.recordOutput("Shooter/FF", ffVoltage);
                Logger.recordOutput("Shooter/TimeVolts", shooterTimeVolts);

                ffVoltage = Math.min(ffVoltage, ShooterConstants.kMaxFeedforwardVoltage);
                shooterMotor.setVoltage(leftFfVoltage); // TODO: Set Back to Middle
                
                
                leftFfVoltage = Math.min(leftFfVoltage, ShooterConstants.kMaxFeedforwardVoltage);
                rightFfVoltage = Math.min(rightFfVoltage, ShooterConstants.kMaxFeedforwardVoltage);
                leftShooterMotor.setVoltage(leftFfVoltage);
                rightShooterMotor.setVoltage(rightFfVoltage);
                
                if (currentRpm > shooterReference) {
                    voltageBuffer.addValue(getVoltage());
                    shooterKv = (voltageBuffer.getAverage() - shooterKs) / shooterReference;
                }
                break;
        }
    }

    private void transitionToHold() {
        controlStage = ControlStage.HOLD;
        double avgVoltage = voltageBuffer.getAverage();
        shooterKv = (avgVoltage - shooterKs) / shooterReference;
        shooterKv = Math.max(0.0, Math.min(shooterKv, 0.01));
        //TODO: Fix transition to hold to account for left and right
    }

    private void resetHoldState() {
        onTarget = false;
        voltageBuffer.clear();
    }

    public static class CircularBuffer {
        private double[] buffer;
        private int numValues = 0;
        
        public CircularBuffer(int size) {
            buffer = new double[size];
        }
        
        public void addValue(double value) {
            if (numValues < buffer.length) {
                buffer[numValues++] = value;
            } else {
                System.arraycopy(buffer, 1, buffer, 0, buffer.length - 1);
                buffer[buffer.length - 1] = value;
            }
        }
        
        public double getAverage() {
            if (numValues == 0) return 0;
            double sum = 0;
            for (int i = 0; i < numValues; i++) {
                sum += buffer[i];
            }
            return sum / numValues;
        }
        
        public int getNumValues() {
            return numValues;
        }
        
        public void clear() {
            numValues = 0;
        }
    }
}