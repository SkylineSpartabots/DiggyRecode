package frc.robot.Commands.Automation;

import java.awt.Robot;
import java.util.Vector;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Drives to a specified pose.
 */
public class AlignToGoal extends Command {
    
    //TODO tune
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            3, 0, 0.02, new TrapezoidProfile.Constraints(Constants.MaxAngularVelocity, Constants.MaxAngularRate), 0.02);

    private CommandSwerveDrivetrain s_Swerve;

    private Translation2d targetGoal;
    private double targetHeading; // rad
    private double ffMinRadius = 0.2, ffMaxRadius = 1.2;

    public AlignToGoal() {
        this.s_Swerve = CommandSwerveDrivetrain.getInstance();

        thetaController.setTolerance(Math.toRadians(4));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        targetGoal = DriverStation.getAlliance().equals(Alliance.Blue)
         ? Constants.FieldConstants.blueGoal.toTranslation2d() : Constants.FieldConstants.redGoal.toTranslation2d();

        SwerveDriveState state = s_Swerve.getState();

        thetaController.setTolerance(Math.toRadians(4));

        thetaController.reset(state.Pose.getRotation().getRadians(),
                state.Speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {

        Pose2d currentPose = s_Swerve.getState().Pose;

        double currentDistance = currentPose.getTranslation().getDistance(targetGoal);
        
        targetHeading = Math.atan2(
            (targetGoal.getY() - currentPose.getY()), (targetGoal.getX() - currentPose.getX())); 

        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetHeading);

        s_Swerve.setControl(new SwerveRequest.FieldCentricFacingAngle()
            .withTargetRateFeedforward((thetaVelocity))
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDesaturateWheelSpeeds(true)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo));
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.setControl(new SwerveRequest.FieldCentricFacingAngle());
    }

    @Override
    public boolean isFinished() {
        return thetaController.atGoal();
    }
}