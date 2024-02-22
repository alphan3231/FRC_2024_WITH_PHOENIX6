package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swerve.BalanceOnBeamCommand;
import frc.robot.commands.swerve.DefaultDriveCommand;
import frc.robot.commands.swerve.ZeroGyroCommand;
import frc.robot.subsystems.DrivebaseSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final Joystick driver = new Joystick(Constants.IOConstants.k_DRIVER_CONTROLLER_PORT);
  private final Joystick operator = new Joystick(Constants.IOConstants.k_OPERATOR_CONTROLLER_PORT);

  private final CommandPS5Controller driverController = new CommandPS5Controller(
      Constants.IOConstants.k_DRIVER_CONTROLLER_PORT);

  public final CommandPS4Controller operatorController = new CommandPS4Controller(
      Constants.IOConstants.k_OPERATOR_CONTROLLER_PORT);

  /* Drive Controls */
  private final int translationAxis = 0;
  private final int strafeAxis = 1;
  private final int rotationAxis = 2;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, 3);
  private final JoystickButton balanceOnBeam = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
  private final JoystickButton robotOrianted = new JoystickButton(driver, 7);
  private final JoystickButton slowMode = new JoystickButton(driver, 6);
  private final JoystickButton slowerMode = new JoystickButton(driver, 5);

  /* Subsystems */ 
  private final DrivebaseSubsystem drivebase = new DrivebaseSubsystem();
 
  private SendableChooser<Command> autoChooser;


  // private final Field2d field;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      // field = new Field2d();

      // PathPlannerLogging.setLogCurrentPoseCallback((pose)-> {
      //   field.setRobotPose(pose);
      // });

      // PathPlannerLogging.setLogTargetPoseCallback((pose) ->{
      //   field.getObject("target pose").setPose(pose);
      // });

      // PathPlannerLogging.setLogActivePathCallback((poses)->{
      //   field.getObject("path").setPoses(poses);
      // });

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto mode", autoChooser);


    drivebase.setDefaultCommand(
        new DefaultDriveCommand(
            drivebase,
            () -> driverController.getRawAxis(translationAxis),
            () -> -driverController.getRawAxis(strafeAxis),
            () -> driverController.getRawAxis(rotationAxis),
            () -> robotOrianted.getAsBoolean(),
            () -> slowMode.getAsBoolean(),
            () -> slowerMode.getAsBoolean()
        ));


    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureButtonBindings() {

    /* Driver Buttons */
    zeroGyro.whileTrue(new ZeroGyroCommand(drivebase));
    balanceOnBeam.whileTrue(new BalanceOnBeamCommand(drivebase));

  
  }

   public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("deneme");

        return AutoBuilder.followPath(path);
    }


  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.IOConstants.k_CONTROLLER_DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
}
