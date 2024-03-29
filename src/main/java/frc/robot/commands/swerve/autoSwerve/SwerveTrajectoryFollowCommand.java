// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.autoSwerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;

public class SwerveTrajectoryFollowCommand extends Command {
  /** Creates a new SwerveTrajectoryFollowCommand. */

  private final DrivebaseSubsystem drivetrain;
  private final Trajectory trajectory;
  private Timer timer = new Timer();
  private Pose2d initialPathPlannerPose = null;

  public SwerveTrajectoryFollowCommand(DrivebaseSubsystem drivetrain, Trajectory trajectory) {

    this.drivetrain = drivetrain;
    this.trajectory = trajectory;

    addRequirements(drivetrain);

  }

  public SwerveTrajectoryFollowCommand(DrivebaseSubsystem drivetrain, String pathFilename, TrajectoryConfig config,
      boolean isReversed, boolean isInitial) {

    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    PathPlannerTrajectory ppTrajectory = PathPlanner.loadPath(pathFilename, config.getMaxVelocity(),
        config.getMaxAcceleration(), isReversed);
    this.trajectory = ppTrajectory;

    if (isInitial) {
      initialPathPlannerPose = new Pose2d(
          ppTrajectory.getInitialPose().getTranslation(),
          ppTrajectory.getInitialState().holonomicRotation);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (initialPathPlannerPose != null) {
      drivetrain.resetOdometry(initialPathPlannerPose);
    }

    timer.reset();
    timer.start();
    // trajectories
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentTime = timer.get();
        // The target state of the trajectory right now (the robot's pose and velocity)
        Trajectory.State targetState = trajectory.sample(currentTime);
        Rotation2d targetRotation = targetState.poseMeters.getRotation();
        // Check if we are using PathPlanner trajectories
        if (trajectory instanceof PathPlannerTrajectory) {
            targetState = ((PathPlannerTrajectory) trajectory).sample(currentTime);
            targetRotation = ((PathPlannerState) targetState).holonomicRotation;
        }

        drivetrain.drive(targetState, targetRotation, true);

        // Pose2d targetPose = targetState.poseMeters;
        // SmartDashboard.putNumber("Target Heading",
        // targetPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Target X", targetPose.getX());
        // SmartDashboard.putNumber("Target Y", targetPose.getY());
  
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (interrupted) drivetrain.drive(new ChassisSpeeds(), true);
        timer.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
