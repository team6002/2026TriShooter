package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIOSim;

public class ShootFuel extends Command{
    private final AbstractDriveTrainSimulation driveSim;
    private int timer;

    public ShootFuel(AbstractDriveTrainSimulation driveSim){
        this.driveSim = driveSim;
    }

    @Override
    public void initialize(){
        timer = 0;
    }

    @Override
    public void execute(){
        if(timer > 15 && IntakeIOSim.numObjectsInHopper() > 0){
            IntakeIOSim.obtainFuelFromHopper();

            //TODO: Update to RebuiltFuelOnFly
            SimulatedArena.getInstance().addGamePieceProjectile(
                new ReefscapeAlgaeOnFly(
                    driveSim.getSimulatedDriveTrainPose().getTranslation(),
                    new Translation2d(Units.inchesToMeters(-16), 0),
                    driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    driveSim.getSimulatedDriveTrainPose().getRotation(), 
                    Inches.of(5),
                    MetersPerSecond.of(7.5),
                    Degrees.of(75)
                )
            );

            timer = 0;
        }else{
            timer ++;
        }
    }

    @Override
    public boolean isFinished(){
        return IntakeIOSim.numObjectsInHopper() <= 0;
    }
}
