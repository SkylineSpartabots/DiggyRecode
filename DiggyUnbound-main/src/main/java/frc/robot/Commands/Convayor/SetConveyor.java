
package frc.robot.Commands.Convayor;

import java.awt.image.ConvolveOp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Conveyor.ConveyorStates;
import frc.robot.Subsystems.Indexer.IndexerStates;;

public class SetConveyor extends Command {
    private Conveyor s_Conveyor;
    ConveyorStates state;

    public SetConveyor(ConveyorStates state) {
        s_Conveyor = Conveyor.getInstance();

        this.state = state;
        addRequirements(s_Conveyor);
    }

    @Override
    public void initialize() {
        s_Conveyor.setVoltage(state.getVoltage());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() { // Return the color sensor result is greater than the threshold or the timer has elapsed, if true end the command
        return true;
    }
}