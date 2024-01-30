package frc.robot.subsystems.intake;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants.IntakeConstants;

public class IntakePID {


    private SparkMaxPIDController flipperController;
    private AbsoluteEncoder encoder;

    private static IntakePID instance;
    
public IntakePID(SparkMaxPIDController systemController, AbsoluteEncoder absEncoder){
        flipperController = systemController;
        encoder = absEncoder;

        flipperController.setFeedbackDevice(encoder);

        flipperController.setP(IntakeConstants.flipperPCoefficient);
        flipperController.setI(IntakeConstants.flipperICoefficient);
        flipperController.setD(IntakeConstants.flipperDCoefficient);
    }

    public double getFlipperPosition(){
        return encoder.getPosition();
    }

    public void setIntakeSP(double setPoint){
        flipperController.setReference(setPoint, ControlType.kPosition);
    }

    public static IntakePID getInstance(SparkMaxPIDController systemController, AbsoluteEncoder absEncoder){
        if (instance == null){
            instance = new IntakePID(systemController,absEncoder);
        }
        return instance;
    }
}
