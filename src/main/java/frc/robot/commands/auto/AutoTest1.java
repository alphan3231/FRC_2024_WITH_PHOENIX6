// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.swerve.autoSwerve.SwerveTrajectoryFollowCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest1 extends SequentialCommandGroup {

  private final DrivebaseSubsystem drivetrain;
  private final TrajectoryConfig defaultConfig;

  public AutoTest1(DrivebaseSubsystem driveTrain) {

    this.drivetrain = driveTrain;
    this.defaultConfig = new TrajectoryConfig(Constants.AutoConstants.k_MAX_SPEED_MPS, Constants.AutoConstants.k_MAX_ACCEL_MPS_SQUARED);


    addCommands(
      new SwerveTrajectoryFollowCommand(driveTrain,"deneme" ,defaultConfig,false,true)
    );


    addRequirements(driveTrain);
  }
}
