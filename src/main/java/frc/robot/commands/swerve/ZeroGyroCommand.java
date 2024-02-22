// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroGyroCommand extends Command {

  private final DrivebaseSubsystem driveBase;

  public ZeroGyroCommand(DrivebaseSubsystem driveBase){
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    driveBase.zeroGyro();
  }
   
  @Override
  public boolean isFinished() {
     return false;
  }
}

