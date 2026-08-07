package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.Radians;

public class DriveControlSystems {

    private double deadbandFactor = 0.85; // higher is more linear joystick controls

    private static CommandSwerveDrivetrain s_Swerve;
    private static Shooter s_Shooter;

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            2.8, 0, 0.1, new TrapezoidProfile.Constraints(Constants.MaxAngularVelocity, Constants.MaxAngularRate), 0.02);

    Boolean mode_AlignToGoal = false;
    Translation2d targetGoal;
    double targetHeading;
    // double ffMinRadius = 0.2, ffMaxRadius = 1.2;

    private static DriveControlSystems instance;

    public static DriveControlSystems getInstance() {
        if(instance == null) {
            instance = new DriveControlSystems();
        }
        return instance;
    }

    public DriveControlSystems() {
        s_Swerve = CommandSwerveDrivetrain.getInstance();
        s_Shooter = Shooter.getInstance();

        thetaController.setTolerance(Math.toRadians(4));
        thetaController.enableContinuousInput(-Math.PI, Math.PI); 
    }

     // =======---===[ ⚙ Joystick processing ]===---========
    public SwerveRequest drive(double driverLY, double driverLX, double driverRX){
        driverLX = scaledDeadBand(driverLX) * Constants.MaxSpeed;
        driverLY = scaledDeadBand(driverLY) * Constants.MaxSpeed;
        driverRX = scaledDeadBand(driverRX) * Constants.MaxAngularRate;

        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            driverLX *= -1;
            driverLY *= -1;
        }

        if (mode_AlignToGoal) {
            SmartDashboard.putBoolean("aiming", true);
            return new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(driverLY)
                .withVelocityY(driverLX)
                .withTargetRateFeedforward((calculateGoalHeading()));
        } else {SmartDashboard.putBoolean("aiming", false);}


        return new SwerveRequest.FieldCentricFacingAngle()
        .withVelocityX(driverLY)
        .withVelocityY(driverLX)
        .withTargetRateFeedforward(driverRX)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDesaturateWheelSpeeds(true)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

        // return new SwerveRequest.RobotCentric()
        // .withVelocityX(driverLY)
        // .withVelocityY(driverLX)
        // .withRotationalRate(driverRX);
    }

    private double calculateGoalHeading() {

        var state = s_Swerve.getState();

        // double airtime = s_Shooter.getAirtime();
        // ChassisSpeeds velocityOffset = state.Speeds.times(airtime);

        // System.out.println(velocityOffset.toString());
        
        // targetHeading = Math.atan2(
        //     (velocityOffset.vyMetersPerSecond + targetGoal.getY() - state.Pose.getY()),
        //     (velocityOffset.vxMetersPerSecond + targetGoal.getX() - state.Pose.getX()));

        targetHeading = Math.atan2(
            (targetGoal.getY() - state.Pose.getY()),
            (targetGoal.getX() - state.Pose.getX()));

        double thetaVelocity = thetaController.getSetpoint().velocity
                + thetaController.calculate(
                        state.Pose.getRotation().getRadians(), targetHeading);

        
        return thetaController.atGoal() ? 0 : thetaVelocity;
    }

    private double scaledDeadBand(double input) {
        if(Math.abs(input) < Constants.stickDeadband) 
            return 0;
        else
            return (deadbandFactor * Math.pow(input, 3)) + (1 - deadbandFactor) * input;
    }

    public void turnOnAutoAim() {
        mode_AlignToGoal = true;

        targetGoal = DriverStation.getAlliance().get().equals(Alliance.Blue)
         ? Constants.FieldConstants.blueGoal.toTranslation2d() : Constants.FieldConstants.redGoal.toTranslation2d();
    }

    public void turnOffAutoAim() {
        mode_AlignToGoal = false;
    }
}
