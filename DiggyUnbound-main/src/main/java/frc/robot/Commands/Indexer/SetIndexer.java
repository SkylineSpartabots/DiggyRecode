
package frc.robot.Commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Indexer.IndexerStates;;

public class SetIndexer extends Command {
    private Indexer s_Indexer;
    IndexerStates state;

    public SetIndexer(IndexerStates state) {
        s_Indexer = Indexer.getInstance();

        this.state = state;
        addRequirements(s_Indexer);
    }

    @Override
    public void initialize() {
        s_Indexer.setVoltage(state.getVoltage());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}