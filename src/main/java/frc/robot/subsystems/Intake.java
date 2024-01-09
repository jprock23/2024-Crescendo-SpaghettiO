package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Ports;
import frc.robot.Constants.IntakeConstants;


public class Intake {

    public enum IntakePosition{
        GROUND(0.0),
        HANDOFF(0.0),
        RETRACTED(0.0);

        private double position;

        private IntakePosition(double position){
            this.position = position;
        }
    }
     
    private CANSparkMax roller;
    private CANSparkMax flipper;

    private SparkMaxPIDController flipperPID;

    private AbsoluteEncoder flipperEncoder;

    private IntakePosition intakePosition = IntakePosition.RETRACTED;
    public static Intake instance;
    
    public Intake() {
        roller = new CANSparkMax(Ports.roller, MotorType.kBrushless);
        roller.setInverted(false);
        roller.burnFlash();
        
        flipper = new CANSparkMax(Ports.flipper,MotorType.kBrushless);
        flipper.setInverted(false);
        flipper.burnFlash();

        flipperPID = flipper.getPIDController();
        flipperEncoder = flipper.getAbsoluteEncoder(Type.kDutyCycle);

        flipperPID.setFeedbackDevice(flipperEncoder);

        flipperPID.setP(IntakeConstants.flipperPCoefficient);
        flipperPID.setI(IntakeConstants.flipperICoefficient);
        flipperPID.setD(IntakeConstants.flipperDCoefficient);

    }

    public void periodic(){
        flipperPID.setReference(intakePosition.position,  CANSparkMax.ControlType.kPosition);
    }

    public void setRollerPower(double power){
        roller.set(power);
    }

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    public double getFlipperPosition(){
        return flipperEncoder.getPosition();
    }

    public boolean hasReachedPose(double tolerance) {
                if (Math.abs(getFlipperPosition() - intakePosition.position) > tolerance) {
            return true;
        }
        return false;
    }

    public void setIntakeState(IntakePosition state) {
        this.intakePosition = state;
    }
    
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }
}