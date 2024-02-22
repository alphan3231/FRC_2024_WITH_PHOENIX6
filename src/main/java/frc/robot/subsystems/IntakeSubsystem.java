// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotID;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax RollerTop = new CANSparkMax(RobotID.Intake.TOP_ROLLER_ID, MotorType.kBrushless); 
  private final CANSparkMax RollerBottom = new CANSparkMax(RobotID.Intake.BOTTOM_ROLLER_ID, MotorType.kBrushless); 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    RollerTop.setInverted(true);
  }

  public void IntakeShoot(double power){
    RollerTop.set(power);
    RollerBottom.set(power);
  }

  public void IntakeOn(double power){
    RollerTop.set(power);
    RollerBottom.set(power);
  }

  public void StopIntake(){
    RollerTop.stopMotor();
    RollerBottom.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
