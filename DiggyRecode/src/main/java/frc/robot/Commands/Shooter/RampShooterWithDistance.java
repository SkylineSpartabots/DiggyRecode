package frc.robot.Commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;

/**
 * Spins the shooter up to the speed the current shot distance asks for.
 * Distance comes from swerve odometry. Quest (the Oculus) feeds that pose
 * through vision measurements; this command does not read the headset directly.
 * The shooter slews the setpoint. This command only publishes the request.
 */
public class RampShooterWithDistance extends Command {
    Shooter s_Shooter;
    CommandSwerveDrivetrain s_Swerve;

    Translation3d targetGoal;

    public RampShooterWithDistance() {
        s_Shooter = Shooter.getInstance();
        s_Swerve = CommandSwerveDrivetrain.getInstance();

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        // getAlliance() is an Optional. Comparing the Optional itself to Alliance.Blue
        // is always false, which used to lock this onto the wrong goal.
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        targetGoal = alliance == Alliance.Blue
                ? Constants.FieldConstants.blueGoal
                : Constants.FieldConstants.redGoal;
    }

    @Override
    public void execute() {
        Translation2d currPose = s_Swerve.getState().Pose.getTranslation();
        double d = currPose.getDistance(targetGoal.toTranslation2d());

        // Empirical fit of flywheel rps vs horizontal distance to the hub.
        double v = 3.52976 * d * d + -5 * d + 36;
        v = Math.min(Constants.shooterMaxRps, Math.max(0, v));

        // Horizontal exit speed is v * radius * 2pi * cos(angle). The old code used
        // v * cos(angle) directly; keep that so airtime stays on the same scale.
        if (v > 1) {
            s_Shooter.updateAirtime(d / (v * Math.cos(Constants.shooterAngleRad)));
        }

        s_Shooter.setVelocity(v);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
