
package frc.robot.Commands.Automation;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeStates;;

public class JiggleBallsDrivetrain extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    Timer timer = new Timer();
    CommandXboxController driver;

    public JiggleBallsDrivetrain(CommandXboxController driver) {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        this.driver = driver;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds();
        if ( (int)(timer.get() / 0.15) % 2 == 0 ) {
            speeds.vxMetersPerSecond = 0.35;
        } else {
            speeds.vxMetersPerSecond = -0.35;
        }
        s_Swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(5) ||
        Math.abs(driver.getLeftX()) > Constants.stickDeadband ||
        Math.abs(driver.getLeftY()) > Constants.stickDeadband;
    }
}