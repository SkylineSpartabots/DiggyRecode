package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor.ConveyorStates;
import frc.robot.Subsystems.Indexer.IndexerStates;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;

public class SetShooterAtMeter extends Command {
    Shooter s_shooter;
    double d;

    public SetShooterAtMeter(double d){
        s_shooter = Shooter.getInstance();

        this.d = d;
        addRequirements(s_shooter);
    }

    @Override
    public void initialize() {
        s_shooter.setVelocity(s_shooter.loggerProEvilEquationOfDoom(d));
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
    }

    
}
