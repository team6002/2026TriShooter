package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;

public class CMD_Intake extends Command{
    private final Intake intake;

    public CMD_Intake(Intake intake){
        this.intake = intake;
        
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setExtenderLowCurrentMode(false);
        intake.setExtenderReference(ExtenderConstants.kExtended);
    }

    @Override
    public void execute(){
        if(intake.getExtenderInPosition()){
            intake.setVoltage(IntakeConstants.kOn);
        }
    }

    @Override
    public void end(boolean interrupted){
        if (interrupted) {
            intake.setExtenderLowCurrentMode(false); 
            intake.setVoltage(IntakeConstants.kOff);
            return;
        }

        intake.setExtenderLowCurrentMode(true);
    }
}
