package frc.robot.subsystems.kicker;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkMax;

public class KickerIOSpark implements KickerIO {
    private final SparkMax kickerMotor;
    private final RelativeEncoder kickerEncoder;
    private final SparkClosedLoopController kickerController;

    private double kickerReference;
    private ControlType kickerType;

    //tuning
    private final LoggedNetworkNumber kS, kV, kP;
    private double lastP = Double.NaN;

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

        //tuning
        kS = new LoggedNetworkNumber("/Tuning/Kicker/kS", KickerConstants.kS);
        kV = new LoggedNetworkNumber("/Tuning/Kicker/kV", KickerConstants.kV);
        kP = new LoggedNetworkNumber("/Tuning/Kicker/kP", KickerConstants.kP);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.kickerReference = Units.radiansToDegrees(getReference());
        inputs.kickerCurrent = getCurrent();
        inputs.kickerVoltage = getVoltage();
        inputs.kickerVelocity = Units.radiansToDegrees(getVelocity());
        inputs.atVelocity = atVelocity();
        inputs.kickerTemp = Fahrenheit.convertFrom(kickerMotor.getMotorTemperature(), Celsius);
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
    public boolean atVelocity(){
        return Math.abs(getReference() - getVelocity()) <= KickerConstants.kTolerance;
    }

    @Override
    public void periodic() {
        //tuning
        // setReference(reference.get());
        double kickerFF = kS.get() + (kV.get() * getReference());

        double p = kP.get();
        if(lastP != p){
            SparkMaxConfig newConfig = new SparkMaxConfig();
            newConfig.apply(KickerConfig.kickerConfig);
            newConfig.closedLoop.pid(p, 0, 0, ClosedLoopSlot.kSlot0);

            kickerMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastP = p;
        }

        // Bypass velocity control at 0 RPM to prevent chatter and allow a smooth coast-down
        if(kickerReference > 0){
            kickerController.setSetpoint(kickerReference, kickerType, ClosedLoopSlot.kSlot0, kickerFF);
        }else{
            kickerController.setSetpoint(0, ControlType.kVoltage, ClosedLoopSlot.kSlot0);
        }
    }
}
