package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotID;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {

    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;


//************************************************************ */
    public SwerveDriveKinematics kinematics = Constants.DrivebaseConstants.SWERVE_KINEMATICS;
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();


//***************************************************************** */
    private Rotation2d lastGivenRotation;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.k_P_THETA_CONTROLLER, 0, 0,
            Constants.AutoConstants.k_THETA_CONTROLLER_CONSTRAINTS);

    private final PIDController xController = new PIDController(Constants.AutoConstants.k_PX_CONTROLLER, 0, 0);
    private final PIDController yController = new PIDController(Constants.AutoConstants.k_PY_CONTROLLER, 0, 0);

    private final HolonomicDriveController pathController = new HolonomicDriveController(
            xController,
            yController,
            thetaController);
    
    private Field2d field = new Field2d();

    private double gyroVal = 0;

    public DrivebaseSubsystem() {

        gyro = new Pigeon2(RobotID.Drivebase.PIGEON_ID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();
        thetaController.enableContinuousInput(Math.PI, -Math.PI);
        pathController.setEnabled(true);

        mSwerveMods = new SwerveModule[] {

            new SwerveModule(1, Constants.DrivebaseConstants.MOD_0.constants),
            new SwerveModule(3, Constants.DrivebaseConstants.MOD_1.constants),
            new SwerveModule(0, Constants.DrivebaseConstants.MOD_2.constants),
            new SwerveModule(2, Constants.DrivebaseConstants.MOD_3.constants)

        };

        //TODO: Set the actual pose
        SmartDashboard.putData(field);
        
        swerveOdometry = new SwerveDrivePoseEstimator(Constants.DrivebaseConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

/************************************************************************************************************** */
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );



    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
/******************************************************************************************************* */
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.DrivebaseConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getGyroscopeRotation()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
                                
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DrivebaseConstants.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

        /**
     * Moves the drivebase around by running the swerve modules.
     * 
     * @param chassisSpeeds The x, y, and theta the drivebase must move in.
     */
    public void drive(ChassisSpeeds chassisSpeeds){

        m_chassisSpeeds = chassisSpeeds;

    }

    /**
     * Moves the drivebase around by running the swerve modules.
     * 
     * @param chassisSpeeds The x, y, and theta the drivebase must move in.
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.DrivebaseConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DrivebaseConstants.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(Trajectory.State targetState, Rotation2d targetRotation, boolean isOpenLoop) {
        // determine ChassisSpeeds from path state and positional feedback control from
        // HolonomicDriveController
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
                getPose(),
                targetState,
                targetRotation);
        // command robot to reach the target ChassisSpeeds
        drive(targetChassisSpeeds, isOpenLoop);
    }

    public void drive(Trajectory.State targetState, Rotation2d targetRotation) {
        // determine ChassisSpeeds from path state and positional feedback control from
        // HolonomicDriveController
        lastGivenRotation = targetRotation;
        ChassisSpeeds targetChassisSpeeds = pathController.calculate(
                getPose(),
                targetState,
                targetRotation);
        // command robot to reach the target ChassisSpeeds
        drive(targetChassisSpeeds);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DrivebaseConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }


    //********************************************************************************************** */

    public ChassisSpeeds getSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);

      }

    
    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.AutoConstants.k_MAX_SPEED_MPS);
    
        for (int i = 0; i < mSwerveMods.length; i++) {
          mSwerveMods[i].setTargetState(targetStates[i]);
        }
      }

    //************************************************************************************ */

    // public void GetEncoderStates(Integer modNumber){
    //     return 
    // }

    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){

        gyro.setYaw(0);

    }

    public Rotation2d getGyroscopeRotation() {
       
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  
      }

    public Rotation2d getYaw() {

        return Rotation2d.fromDegrees(gyro.getYaw().getValue());

    }

    public double getGyroDouble(){

        return Math.IEEEremainder(gyro.getAngle(), 360); 
        //* (Constants.kGyroReversed ? -1.0 : 1.0)

    }

    public double getGyroUltimate(){

        return Math.abs((gyro.getYaw().getValue()) % 360) - 180;
    }

    public double getYawAsDouble() {

        return gyro.getYaw().getValue();

    }

    public double getPitchAsDouble() {

        return gyro.getPitch().getValue();

    }

    public double getRollAsDouble(){

        return gyro.getRoll().getValue() + 1.5;

    }

    @Override
    public void periodic(){

        swerveOdometry.update(getGyroscopeRotation(), getModulePositions());

        SmartDashboard.putNumber( "Pigeon Yaw", getYawAsDouble());

        SmartDashboard.putNumber("Gyro Double:", getGyroUltimate());

        SmartDashboard.putNumber( "Pigeon Roll", getRollAsDouble());

        SmartDashboard.putNumber( "Pigeon Pitch", getPitchAsDouble());

        field.setRobotPose(getPose());


        for(SwerveModule mod : mSwerveMods){

            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
               
        }

    }
}