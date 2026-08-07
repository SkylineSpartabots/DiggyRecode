
package frc.robot.Commands.Pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Pivot.PivotStates;;

public class SetPivot extends Command {
    private Pivot s_Pivot;
    PivotStates state;


    public SetPivot(PivotStates state) {
        s_Pivot = Pivot.getInstance();

        this.state = state;

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        s_Pivot.setVoltage(state.getVoltage());
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