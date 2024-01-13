package frc.robot.subsystems.launcher;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.LauncherConstants;

public class LauncherPID {

    private SparkMaxPIDController angleController1;
    private SparkMaxPIDController angleController2;

    private SparkMaxPIDController voltageController1;
    private SparkMaxPIDController voltageController2;

    private SparkMaxPIDController flickerController;

    private AbsoluteEncoder angleEncoder1;
    private AbsoluteEncoder angleEncoder2;

    private LauncherPID instance;

    public LauncherPID(CANSparkMax launchMotor1, CANSparkMax launchMotor2, CANSparkMax launcherAngle1, CANSparkMax launcherAngle2, CANSparkMax flicker){
        voltageController1 = launchMotor1.getPIDController();
        voltageController2 = launchMotor2.getPIDController();

        voltageController1.setP(LauncherConstants.launchPCoefficient);
        voltageController1.setI(LauncherConstants.launchICoefficient);
        voltageController1.setD(LauncherConstants.launchDCoefficient);
         
        voltageController2.setP(LauncherConstants.launchPCoefficient);
        voltageController2.setI(LauncherConstants.launchICoefficient);
        voltageController2.setD(LauncherConstants.launchDCoefficient);

        voltageController1.setFeedbackDevice(launchMotor1.getEncoder());
        voltageController2.setFeedbackDevice(launchMotor2.getEncoder());

        angleController1 = launcherAngle1.getPIDController();
        angleController2 = launcherAngle2.getPIDController();

        angleEncoder1 = launcherAngle1.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder2 = launcherAngle2.getAbsoluteEncoder(Type.kDutyCycle);

        angleController1.setP(LauncherConstants.anglePCoefficient);
        angleController1.setI(LauncherConstants.angleICoefficient);
        angleController1.setD(LauncherConstants.angleDCoefficient);

        angleController1.setP(LauncherConstants.anglePCoefficient);
        angleController1.setI(LauncherConstants.angleICoefficient);
        angleController1.setD(LauncherConstants.angleDCoefficient);

        angleController1.setFeedbackDevice(angleEncoder1);
        angleController2.setFeedbackDevice(angleEncoder2);

        flickerController = flicker.getPIDController();

        flickerController.setP(LauncherConstants.anglePCoefficient);
        flickerController.setI(LauncherConstants.angleICoefficient);
        flickerController.setD(LauncherConstants.angleDCoefficient);

        flickerController.setFeedbackDevice(flicker.getEncoder());
    }

    public void setVoltageSP(double setPoint){
        voltageController1.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
        voltageController2.setReference(setPoint, CANSparkMax.ControlType.kVoltage);
    }

    public void setAngleSP(double setPoint){
        angleController1.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        angleController2.setReference(setPoint, CANSparkMax.ControlType.kPosition);

    }
    
    public void setFlickerSP(double setPoint){
        flickerController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }

    public LauncherPID getInstance(CANSparkMax launchMotor1, CANSparkMax launchMotor2, CANSparkMax launcherAngle1, CANSparkMax launcherAngle2, CANSparkMax flicker){
        if(instance == null){
            instance = new LauncherPID(launchMotor1, launchMotor2, launcherAngle1, launcherAngle2, flicker);
        }

        return instance;
    }

}
