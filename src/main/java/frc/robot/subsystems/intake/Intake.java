package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;
import frc.robot.subsystems.intake.IntakeStates.*;

public class Intake {
     
    private CANSparkMax roller;
    private CANSparkMax flipper;

    // private IntakePID control;

    private IntakePosition intakePosition = IntakePosition.RETRACTED;
    public static Intake instance;

    private double power = .5;
    private double flip = 0.25;

    private ArmFeedforward feedforward;
    private SparkMaxPIDController flipperController;

    private AbsoluteEncoder encoder;
    
    public Intake() {
        roller = new CANSparkMax(Ports.roller, MotorType.kBrushless);
        roller.restoreFactoryDefaults();

        roller.setSmartCurrentLimit(40);
        roller.setIdleMode(IdleMode.kCoast);
        roller.setInverted(false);
        roller.burnFlash();
        
        flipper = new CANSparkMax(Ports.flipper,MotorType.kBrushless);
        flipper.restoreFactoryDefaults();

        flipper.setSmartCurrentLimit(70);
        flipper.setIdleMode(IdleMode.kBrake);
        flipper.setInverted(true);
        flipper.burnFlash();

        encoder = flipper.getAbsoluteEncoder(Type.kDutyCycle);

        feedforward = new ArmFeedforward(0, 0.031, 0, 0);

        flipperController = flipper.getPIDController();
        flipperController.setFeedbackDevice(encoder);

        flipperController.setP(IntakeConstants.flipperPCoefficient);
        flipperController.setI(IntakeConstants.flipperICoefficient);
        flipperController.setD(IntakeConstants.flipperDCoefficient);
        flipperController.setFF(feedforward.calculate(.36, 0));

        //.36

        // control = IntakePID.getInstance(flipper.getPIDController(), flipper.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic(){

        flipperController.setReference(.36, ControlType.kDutyCycle);
        // flipper.set(feedforward.calculate(.36, 0));
        }

    public void reverseFlipper(){
        flipper.set(-flip);
    }

    public void setRollerPower(){
        roller.set(power);
    }

    public void setReverseRollerPower(){
        roller.set(-power);
    }

    public void setFlipperPower(){
        flipper.set(flip);
    }

    public void setFlipperOff(){
        flipper.set(0.0);
    }

    public void setRollerOff(){
        roller.set(0);
    }

    public void decreaseRoller(){
        power -= power;
    }

        public void increaseRoller(){
        power += power;
    }

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    public double getFlipperPosition(){
        return flipper.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        }

    // public boolean hasReachedPose(double tolerance) {
    //             if (Math.abs(control.getFlipperPosition() - intakePosition.position) > tolerance) {
    //         return true;
    //     }
    //     return false;
    // }

    public void setIntakeState(IntakePosition state) {
        this.intakePosition = state;
    }
    
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }
}