
package frc.robot.Commands.Pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Intake.IntakeStates;

public class ForcePivot extends Command {
    private Pivot s_Pivot;
    Timer timer = new Timer();
    double volts;

    public ForcePivot() {
        s_Pivot = Pivot.getInstance();
        volts = -4.5;

        addRequirements(s_Pivot);
    }

    public ForcePivot(double volts) {
        s_Pivot = Pivot.getInstance();

        this.volts = volts;

        addRequirements(s_Pivot);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        s_Pivot.setVoltage(volts);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        s_Pivot.setVoltage(0);
        // s_Pivot.zeroPivot();
        // s_Pivot.setRotations(0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}