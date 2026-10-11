package frc.robot.Commands.Automation;

import java.util.function.DoubleSupplier;

import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.DriveControlSystems;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Faces the alliance hub while this command is running.
 * Translation comes from the suppliers: driver sticks in teleop, zero at the end of auto.
 * Finishes when auto-aim is turned off (right trigger, or the operator kill).
 */
public class AlignToGoal extends Command {

    private final CommandSwerveDrivetrain s_Swerve;
    private final DriveControlSystems control;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;

    public AlignToGoal(DoubleSupplier forward, DoubleSupplier strafe) {
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.control = DriveControlSystems.getInstance();
        this.forward = forward;
        this.strafe = strafe;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        control.turnOnAutoAim();
    }

    @Override
    public void execute() {
        // drive() aims while auto-aim is on, and still applies these translation sticks.
        s_Swerve.setControl(control.drive(forward.getAsDouble(), strafe.getAsDouble(), 0));
    }

    @Override
    public void end(boolean interrupted) {
        control.turnOffAutoAim();
    }

    @Override
    public boolean isFinished() {
        return !control.isAutoAiming();
    }
}
