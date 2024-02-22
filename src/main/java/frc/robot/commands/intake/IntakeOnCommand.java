package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOnCommand extends Command {

  private final IntakeSubsystem mIntakeSubsystem;

  public IntakeOnCommand(IntakeSubsystem intakeSubsystem) {

    this.mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSubsystem.IntakeOn(Constants.IntakeConstants.INTAKE_ON_POWER);
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
