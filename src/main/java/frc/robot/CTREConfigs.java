package frc.robot;



import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;


public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){

        // Cancoder Config
        swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.DrivebaseConstants.CAN_CODER_INVERT;


        // Angle Motor Configs
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.DrivebaseConstants.ANGLE_MOTOR_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.DrivebaseConstants.ANGLE_NEUTRAL_MODE;

        swerveAngleFXConfig.Slot0.kP = Constants.DrivebaseConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.DrivebaseConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.DrivebaseConstants.angleKD;
        
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.DrivebaseConstants.ANGLE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.DrivebaseConstants.ANGLE_CONT_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.DrivebaseConstants.ANGLE_PEAK_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.DrivebaseConstants.ANGLE_PEAK_CURRENT_DURATION;

        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.DrivebaseConstants.ANGLE_GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        //Drive Motor Configs
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.DrivebaseConstants.DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.DrivebaseConstants.DRIVE_NEUTRAL_MODE;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.DrivebaseConstants.DRIVE_GEAR_RATIO;

        swerveAngleFXConfig.Slot0.kP = Constants.DrivebaseConstants.driveKP;
        swerveAngleFXConfig.Slot0.kI = Constants.DrivebaseConstants.driveKI;
        swerveAngleFXConfig.Slot0.kD = Constants.DrivebaseConstants.driveKD;
        
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.DrivebaseConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.DrivebaseConstants.DRIVE_CONT_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.DrivebaseConstants.DRIVE_PEAK_CURRENT_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.DrivebaseConstants.DRIVE_PEAK_CURRENT_DURATION;


        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.DrivebaseConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.DrivebaseConstants.OPEN_LOOP_RAMP;
    }
    
}