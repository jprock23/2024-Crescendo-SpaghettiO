package frc.robot.subsystems.intake;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants.IntakeConstants;

public class IntakePID {


    private SparkMaxPIDController flipperController;
    private AbsoluteEncoder encoder;

    private static IntakePID instance;
    
public IntakePID(CANSparkMax flipper){
        flipperController = flipper.getPIDController();
        encoder = flipper.getAbsoluteEncoder(Type.kDutyCycle);

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

    public static IntakePID getInstance(CANSparkMax flipper){
        if (instance == null){
            instance = new IntakePID(flipper);
        }
        return instance;
    }
}
