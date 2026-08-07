
package frc.robot.Commands.Pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Intake.IntakeStates;
import frc.robot.Subsystems.Pivot.PivotStates;;

public class SetPivotTimed extends Command {
    private Pivot s_Pivot;
    PivotStates state;

    Timer timer;


    public SetPivotTimed(PivotStates state) {
        s_Pivot = Pivot.getInstance();

        this.state = state;

        timer = new Timer();

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        timer.restart();
        s_Pivot.setVoltage(state.getVoltage());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        s_Pivot.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.2);
    }
}