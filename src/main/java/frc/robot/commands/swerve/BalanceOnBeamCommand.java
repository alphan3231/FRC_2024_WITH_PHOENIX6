package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class BalanceOnBeamCommand extends Command {

    private DrivebaseSubsystem driveBase;

    private double errorY;
    private double currentAngleY;
    private double drivePowerY;
    private double errorX;
    private double currentAngleX;
    private double drivePowerX;

    
public BalanceOnBeamCommand(DrivebaseSubsystem driveBase) {

    this.driveBase = driveBase;
    addRequirements(driveBase);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.currentAngleY = driveBase.getPitchAsDouble();
    this.currentAngleX = driveBase.getRollAsDouble();

    errorY = Constants.DrivebaseConstants.BEAM_BALANCED_GOAL_DEGREES - currentAngleY;
    drivePowerY = Constants.DrivebaseConstants.BEAM_BALANACED_DRIVE_KP * errorY;

    errorX = Constants.DrivebaseConstants.BEAM_BALANCED_GOAL_DEGREES - currentAngleX;
    drivePowerX = Constants.DrivebaseConstants.BEAM_BALANACED_DRIVE_KP * errorX;
    
    if (drivePowerY < 0) {
      drivePowerY *= Constants.DrivebaseConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

    if (drivePowerX < 0) {
      drivePowerX *= Constants.DrivebaseConstants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

    // Limit the max power
    if (Math.abs(drivePowerY) > 0.1) {
      drivePowerY = Math.copySign(0.1, drivePowerY);
    }

    if (Math.abs(drivePowerX) > 0.5) {
      drivePowerX = Math.copySign(0.5, drivePowerX);
    }

    driveBase.drive(new ChassisSpeeds(drivePowerX, 0, 0.0), true);

    SmartDashboard.putNumber("Current Angle Y (Pitch): ", currentAngleY);
    SmartDashboard.putNumber("Error Y", errorY);
    SmartDashboard.putNumber("Drive Power Y: ", drivePowerY);

    SmartDashboard.putNumber("Current Angle X (Roll): ", currentAngleX);
    SmartDashboard.putNumber("Error X ", errorX);
    SmartDashboard.putNumber("Drive Power X: ", drivePowerX);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveBase.drive(new ChassisSpeeds(0, 0, 0), true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (Math.abs(errorX) < Constants.DrivebaseConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES) && (Math.abs(errorY) < Constants.DrivebaseConstants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES); // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  
}




















}
