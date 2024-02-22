package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class DefaultDriveCommand extends Command {   

    private final DrivebaseSubsystem drivebase;
        
    private DoubleSupplier xTranslationSup;
    private DoubleSupplier yTranslationSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotOriantedSup;
    private BooleanSupplier slowMode;
    private BooleanSupplier slowerMode;

    private final double SLOW_MODE_MULTIPLIER = 0.2;
    private final double SLOWER_MODE_MULTIPLIER = 0.4;

    public DefaultDriveCommand(DrivebaseSubsystem drivebase, DoubleSupplier xTranslationSup, DoubleSupplier yTranslationSup,
         DoubleSupplier rotationSup, BooleanSupplier robotOriantedSup, BooleanSupplier slowMode, BooleanSupplier slowerMode) {
        
        this.drivebase = drivebase;
        addRequirements(drivebase);

        this.xTranslationSup = xTranslationSup;
        this.yTranslationSup = yTranslationSup;
        this.rotationSup = rotationSup;
        this.robotOriantedSup = robotOriantedSup;
        this.slowMode = slowMode;
        this.slowerMode = slowerMode;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double xTranslationVal = MathUtil.applyDeadband(xTranslationSup.getAsDouble(), Constants.DrivebaseConstants.STICK_DEADBAND);
        double yTranslationVal = MathUtil.applyDeadband(yTranslationSup.getAsDouble(), Constants.DrivebaseConstants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.DrivebaseConstants.STICK_DEADBAND);
        
        // if (rotationVal == 0){
        //     rotationVal = 0;
        // } else if (rotationVal < 0){
        //     rotationVal += 180;
        // } else if (rotationVal > 0){
        //     rotationVal -= 180;
        // }


        // if (rotationVal < 0){
        //     if (rotationVal < -180){
        //         rotationVal += 180;
        //     } else if(rotationVal > -180){
        //         rotationVal -= 180;
        //     }
        // } else if (rotationVal == 0){
        //     rotationVal = 0;
    
        // } else if (rotationVal > 180){
        //     rotationVal -= 180;
    
        // } else if (rotationVal <= 180){
        //     rotationVal +=180;
        // }




        if (slowMode.getAsBoolean()) {
            xTranslationVal *= SLOW_MODE_MULTIPLIER;
            yTranslationVal *=  SLOW_MODE_MULTIPLIER;
            rotationVal *=  SLOW_MODE_MULTIPLIER;
        } else if(slowerMode.getAsBoolean()){
            xTranslationVal *= SLOWER_MODE_MULTIPLIER;
            yTranslationVal *=  SLOWER_MODE_MULTIPLIER;
            rotationVal *=  SLOWER_MODE_MULTIPLIER;

        }
        



        //*********************************************** */
        //*rotationVal e -180 veya + 180 vercek bir kısım */


        //*********************************************** */
        
        /* Drive */
        drivebase.drive(
            new Translation2d(xTranslationVal, yTranslationVal).times(Constants.DrivebaseConstants.MAX_SPEED), 
            rotationVal * Constants.DrivebaseConstants.MAX_ANGULAR_VELOCITY,
            !robotOriantedSup.getAsBoolean(),
            true
        );
    }
}