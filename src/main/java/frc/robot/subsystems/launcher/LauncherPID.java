package frc.robot.subsystems.launcher;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.LauncherConstants;

public class LauncherPID {

    private SparkMaxPIDController bigFlipController1;
    private SparkMaxPIDController bigFlipController2;

    private SparkMaxPIDController voltageController1;
    private SparkMaxPIDController voltageController2;

    private SparkMaxPIDController flickerController;

    private AbsoluteEncoder bigFlipEncoder1;
    private AbsoluteEncoder bigFlipEncoder2;

    private LauncherPID instance;

    public LauncherPID(SparkMaxPIDController vol1, SparkMaxPIDController vol2, RelativeEncoder volEncoder1, RelativeEncoder volencoder2, 
    SparkMaxPIDController bigFlip1, SparkMaxPIDController bigFlip2, AbsoluteEncoder flipEncoder1, AbsoluteEncoder flipEncoder2, 
    SparkMaxPIDController flick, AbsoluteEncoder flickEncoder){
        voltageController1 = vol1;
        voltageController2 = vol2;

        voltageController1.setP(LauncherConstants.launchPCoefficient);
        voltageController1.setI(LauncherConstants.launchICoefficient);
        voltageController1.setD(LauncherConstants.launchDCoefficient);
         
        voltageController2.setP(LauncherConstants.launchPCoefficient);
        voltageController2.setI(LauncherConstants.launchICoefficient);
        voltageController2.setD(LauncherConstants.launchDCoefficient);

        voltageController1.setFeedbackDevice(volEncoder1);
        voltageController2.setFeedbackDevice(volencoder2);

        bigFlipController1 = bigFlip1;
        bigFlipController2 = bigFlip2;

        bigFlipEncoder1 = flipEncoder1;
        bigFlipEncoder2 = flipEncoder2;

        bigFlipController1.setP(LauncherConstants.pivotPCoefficient);
        bigFlipController1.setI(LauncherConstants.pivotICoefficient);
        bigFlipController1.setD(LauncherConstants.pivotDCoefficient);

        bigFlipController1.setP(LauncherConstants.pivotPCoefficient);
        bigFlipController1.setI(LauncherConstants.pivotICoefficient);
        bigFlipController1.setD(LauncherConstants.pivotDCoefficient);

        bigFlipController1.setFeedbackDevice(bigFlipEncoder1);
        bigFlipController2.setFeedbackDevice(bigFlipEncoder2);

        flickerController = flick;

        flickerController.setP(LauncherConstants.pivotPCoefficient);
        flickerController.setI(LauncherConstants.pivotICoefficient);
        flickerController.setD(LauncherConstants.pivotDCoefficient);

        flickerController.setFeedbackDevice(flickEncoder);
    }

    public void setVoltageSP(double setPoint){
        voltageController1.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
        voltageController2.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
    }

    public void setAngleSP(double setPoint){
        bigFlipController1.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        bigFlipController2.setReference(setPoint, CANSparkMax.ControlType.kPosition);

    }
    
    public void setFlickerSP(double setPoint){
        flickerController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }

    public LauncherPID getInstance(SparkMaxPIDController vol1, SparkMaxPIDController vol2, RelativeEncoder volEncoder1, RelativeEncoder volencoder2, 
    SparkMaxPIDController bigFlip1, SparkMaxPIDController bigFlip2, AbsoluteEncoder flipEncoder1, AbsoluteEncoder flipEncoder2, 
    SparkMaxPIDController flick, AbsoluteEncoder flickEncoder){
        if(instance == null){
            instance = new LauncherPID(vol1, vol2, volEncoder1, volencoder2, 
     bigFlip1, bigFlip2, flipEncoder1, flipEncoder2, flick, flickEncoder);
        }

        return instance;
    }

}
