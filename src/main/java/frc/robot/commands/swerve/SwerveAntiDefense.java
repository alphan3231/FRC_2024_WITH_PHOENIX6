package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;

public class SwerveAntiDefense extends Command {

  private DrivebaseSubsystem drivebase;

  private SwerveModuleState desiredStates[] = {

    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(-45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(45)),
    new SwerveModuleState(0.0011, Rotation2d.fromDegrees(-45)),
    
  };

  public SwerveAntiDefense(DrivebaseSubsystem drivebase) {

    this.drivebase = drivebase;

    addRequirements(drivebase);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivebase.setModuleStates(desiredStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
}
