// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Quest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowChoreoTrajectory extends Command {
  private Optional<Trajectory<SwerveSample>> trajectoryOpt;
  private Trajectory<SwerveSample> trajectory = null;

  private Optional<DriverStation.Alliance> alliance;
  private CommandSwerveDrivetrain s_Swerve;
  private Quest quest;
  private Optional<Pose2d> startPose;
  private Timer timer;
  
  private PIDController xController = new PIDController(0.67, 0, 0);
  private PIDController yController = new PIDController(0.67, 0, 0);
  private static final PIDController thetaController = new PIDController(2.7, 0, 0.02); //tuned. -ethan


  public FollowChoreoTrajectory(Optional<Trajectory<SwerveSample>> traj) {
    // System.out.println(traj);

    this.quest = Quest.getInstance();

    s_Swerve = CommandSwerveDrivetrain.getInstance();
    alliance = DriverStation.getAlliance();
    timer = new Timer();

    trajectoryOpt = traj;
    
    addRequirements(s_Swerve);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    if (trajectoryOpt.isPresent()) {
      trajectory = trajectoryOpt.get();
      System.out.println(" trajectory optinal present");
    }


    if (trajectory != null){
      startPose = trajectory.getInitialPose(alliance.get() == DriverStation.Alliance.Red);
      s_Swerve.resetOdo(startPose.get());
      quest.anchorQuest(new Pose3d(startPose.get()));
    }
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(trajectory != null){
      Optional<SwerveSample> sample = trajectory.sampleAt(timer.get(), alliance.get() == DriverStation.Alliance.Red);
      FollowChoreoSample(sample.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Pose2d pose = s_Swerve.getState().Pose;
    s_Swerve.setControl(new SwerveRequest.ApplyFieldSpeeds().withSpeeds(new ChassisSpeeds()));

    if (trajectory != null){
      Optional<Pose2d> goal = trajectory.getFinalPose(alliance.get() == DriverStation.Alliance.Red);
  }}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(trajectory);
    return trajectory != null ? timer.hasElapsed(trajectory.getTotalTime()+ 0.2) : true;
  }

    public void FollowChoreoSample(SwerveSample sample) {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Get the current pose of the robot
        Pose2d pose = s_Swerve.getState().Pose;

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + thetaController.calculate(pose.getRotation().getRadians(), sample.heading)
            );

        // ff
        // ChassisSpeeds speeds = new ChassisSpeeds(
        //     sample.vx,
        //     sample.vy,
        //     0
        // );

            // Apply the generated speeds
        s_Swerve.setControl(
            new SwerveRequest.FieldCentric()
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
            // .withSteerRequestType(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo)
            // .withDesaturateWheelSpeeds(true)
        );

    }

}