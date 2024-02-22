package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpeakerShootCommand extends Command {

  private final IntakeSubsystem mIntakeSubsystem;

  public IntakeSpeakerShootCommand(IntakeSubsystem intakeSubsystem) {
    
    this.mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSubsystem.IntakeShoot(Constants.IntakeConstants.SPEAKER_SHOOT_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntakeSubsystem.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
