package frc.robot;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;

import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final class DrivebaseConstants {

    /* Module Specific Constants */
    /* Front Left Module - Module 0 Sari */
    public static final class MOD_0 { // TODO: This must be tuned to specific robot
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.9-90);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
          RobotID.Drivebase.Mod0.DRIVE_MOTOR_ID,
          RobotID.Drivebase.Mod0.ANGLE_MOTOR_ID,
          RobotID.Drivebase.Mod0.CANCODER_ID,
          angleOffset);
    }

    /* Front Right Module - Module 1  Kirmizi
    */
    public static final class MOD_1 { // TODO: This must be tuned to specific robot
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(344.7-90); //35.5 //254.7
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
          RobotID.Drivebase.Mod1.DRIVE_MOTOR_ID,
          RobotID.Drivebase.Mod1.ANGLE_MOTOR_ID,
          RobotID.Drivebase.Mod1.CANCODER_ID,
          angleOffset);
    }

    /* Back Left Module - Module 2  gri*/
    public static final class MOD_2 { // TODO: This must be tuned to specific robot
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(330.2-90);
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(
          RobotID.Drivebase.Mod2.DRIVE_MOTOR_ID,
          RobotID.Drivebase.Mod2.ANGLE_MOTOR_ID,
          RobotID.Drivebase.Mod2.CANCODER_ID,
          angleOffset);
    }

    /* Back Right Module - Module 3 yeşil */
    public static final class MOD_3 { // TODO: This must be tuned to specific robot
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(244.68-90);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(
          RobotID.Drivebase.Mod3.DRIVE_MOTOR_ID,
          RobotID.Drivebase.Mod3.ANGLE_MOTOR_ID,
          RobotID.Drivebase.Mod3.CANCODER_ID,
          angleOffset);
    }

    public static final COTSFalconSwerveConstants CHOSEN_MODULE = COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L3);


    /* Drivetrain Constants */
    public static final double TRACKWIDTH = 0.487;
    public static final double WHEELBASE = 0.487;
    public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_SPEED = 4.5; // TODO: This must be tuned to specific robot

    public static final double MAX_ANGLE = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double MAX_ANGULAR_VELOCITY = 20; // TODO: This must be tuned to specific robot

    /* Slowmode multipliers */
    public static final double slowmodeMultiplier = 0.4; // TODO: Modify for drivers preference
    public static final double slowermodeMultiplier = 0.8; // TODO: Modify for drivers preference
    
    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEELBASE / 2.0, TRACKWIDTH / 2.0),
        new Translation2d(WHEELBASE / 2.0, -TRACKWIDTH / 2.0),
        new Translation2d(-WHEELBASE / 2.0, TRACKWIDTH / 2.0),
        new Translation2d(-WHEELBASE / 2.0, -TRACKWIDTH / 2.0));

    /* Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
    public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CAN_CODER_INVERT = CHOSEN_MODULE.canCoderInvert;

    /* Angle Motor PID Values */
    public static final double angleKP = CHOSEN_MODULE.angleKP;
    public static final double angleKI = CHOSEN_MODULE.angleKI;
    public static final double angleKD = CHOSEN_MODULE.angleKD;
    public static final double angleKF = CHOSEN_MODULE.angleKF;

    public static final int angleCurrentLimit = 25;
    

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Neutral Modes */
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final int PIGEON_ID = 13;

    public static final double STICK_DEADBAND = 0.1;

    /* Swerve Current Limiting */
    // TODO: These values must be tuned for this robot
    // TEST ON ROBOT
    public static final int ANGLE_CONT_CURRENT_LIMIT = 25;
    public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
    public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

    public static final int DRIVE_CONT_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    public static final double BEAM_BALANACED_DRIVE_KP = 0.5; // P (Proportional) constant of a PID loop
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1;

    public static final double TRACKED_TARGET_ROTATION_KP = 0.165;

  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                                            // to specific robot
    public static final double k_MAX_SPEED_MPS = 3;
    public static final double k_MAX_ACCEL_MPS_SQUARED = 2;
    public static final double k_MAX_ANGULAR_SPEED_RADS_PER_SEC = Math.PI;
    public static final double k_MAX_ANGULAR_ACCEL_RADS_PER_SEC_SQUARED = Math.PI;

    public static final double k_PX_CONTROLLER = 5;
    public static final double k_PY_CONTROLLER = 5;
    public static final double k_P_THETA_CONTROLLER = 4.5;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints k_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        k_MAX_ANGULAR_SPEED_RADS_PER_SEC, k_MAX_ANGULAR_ACCEL_RADS_PER_SEC_SQUARED);

  }

  public static final class LimelightConstants {
    
    public static final double MOUNTED_ANGLE = 250; //degrees

    public static final double TARGET_HEIGHT = Units.metersToInches(1.33); 

    public static final double LENS_HEIGHT = Units.metersToInches(1.35);

    public static final double TRACKED_TAG_ROATION_KP = 0; 

    }

  public static final class IntakeConstants {
    public static final double AMP_SHOOT_POWER = 0.8; // TODO: this might be tuned specific
    public static final double SPEAKER_SHOOT_POWER = 1.0; // TODO: this might be tuned specific
    public static final double INTAKE_ON_POWER = 0.2; // TODO: this might be tuned specific
  }


  public static final class IOConstants {
    
    public static final int k_DRIVER_CONTROLLER_PORT = 0;
    public static final int k_OPERATOR_CONTROLLER_PORT = 1;
    public static final double k_CONTROLLER_DEADBAND = 0.05;

  }

}
