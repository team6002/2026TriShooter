package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;

public class ShooterController {
    public enum ControlStage {
        IDLE,
        SPIN_UP,
        HOLD_WHEN_READY,
        HOLD
    }

    private ControlStage controlStage = ControlStage.IDLE;
    private double targetRpm = 0;
    private double timeOnTarget = 0;
    
    private double shooterKs, shooterKv, shooterVoltRamp, shootingTimeStarted;
    private boolean shooting = false;

    public ShooterController(double shooterKs, double shooterKv, double shooterVoltRamp){
        this.shooterKs = shooterKs;
        this.shooterKv = shooterKv;
        this.shooterVoltRamp = shooterVoltRamp;
    }

    public void setTargetRpm(double rpm) {
        if (targetRpm != rpm) {
            targetRpm = rpm;
            controlStage = ControlStage.SPIN_UP;
        }
    }

    public double getTargetRpm(){
        return targetRpm;
    }

    public void startShooting(){
        shootingTimeStarted = Timer.getFPGATimestamp();
        shooting = true;
    }

    public void update(double currentRpm){
        switch (controlStage) {
            case IDLE:
                break;
            case SPIN_UP:
                if (Math.abs(currentRpm - targetRpm) < ShooterConstants.kStartOnTargetVel){
                    controlStage = ControlStage.HOLD_WHEN_READY;
                    timeOnTarget = Timer.getFPGATimestamp();
                }
                break;
            case HOLD_WHEN_READY:
                // if at target velocity, wait for velocity to hold steady for a short time, if velocity drops reset timer
                if(Math.abs(currentRpm - targetRpm) < ShooterConstants.kStartOnTargetVel){
                    if((Timer.getFPGATimestamp() - timeOnTarget) > .25){
                        controlStage = ControlStage.HOLD;
                    }
                }else{
                    timeOnTarget = Timer.getFPGATimestamp();
                }
                break;
            case HOLD:
                if(currentRpm - targetRpm > ShooterConstants.kStopOnTargetVel){
                    // shootingTimeStarted = Timer.getFPGATimestamp();
                }
                break;
            default:
                break;
        }
    }

    public double getVoltageCommand() {
        switch(controlStage) {
            case IDLE:
                return 0;
            case HOLD:
                double addedVoltRamp = 0;
                if(shooting){
                    double elapsedTime = Timer.getFPGATimestamp() - shootingTimeStarted;
                    //delay ramping of voltage until after the first ball goes through
                    if(elapsedTime > .1){
                        addedVoltRamp = elapsedTime * shooterVoltRamp;
                    }
                }
                return Math.min(shooterKs + (shooterKv * targetRpm) + addedVoltRamp, 12);
            default:
                return shooterKs + (shooterKv * targetRpm);
        }
    }

    public void setVoltageRamp(double shooterVoltRamp){
        this.shooterVoltRamp = shooterVoltRamp;
    }

    public boolean isReady() {
        return controlStage == ControlStage.HOLD;
    }
    
    public ControlStage getControlStage() {
        return controlStage;
    }

    public void setSpinUp(){
        controlStage = ControlStage.SPIN_UP;
    }

    public void stop() {
        controlStage = ControlStage.IDLE;
        targetRpm = 0;
        shooting =  false;
    }
}
