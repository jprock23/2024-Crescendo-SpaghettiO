package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;

public class Intake {

     public enum IntakePosition{
        GROUND(0.492),
        HANDOFF(0.0699);

        public double position;

        private IntakePosition(double position){
            this.position = position;
        }
     }
     
    private CANSparkMax roller;
    private CANSparkMax flipper;

    // private IntakePID control;

    private IntakePosition intakePosition = IntakePosition.HANDOFF;
    public static Intake instance;

    private double power = .5;
    private double flip = 0.25;

    private ArmFeedforward feedforward;
    // private SparkMaxPIDController flipperController;
    private PIDController flipperController;

    private AbsoluteEncoder encoder;

    private double flipperPower = 0.0;

    
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
        encoder.setPositionConversionFactor(25);

        feedforward = new ArmFeedforward(0, 0.031, 0, 0);

        // flipperController = flipper.getPIDController();
        // flipperController.setFeedbackDevice(encoder);

        flipperController = new PIDController(IntakeConstants.flipperPCoefficient, IntakeConstants.flipperICoefficient, IntakeConstants.flipperDCoefficient);

        // flipperController.setP(IntakeConstants.flipperPCoefficient);
        // flipperController.setI(IntakeConstants.flipperICoefficient);
        // flipperController.setD(IntakeConstants.flipperDCoefficient);
        // flipperController.setFF(feedforward.calculate(.38, 0));

        //.36


        // control = IntakePID.getInstance(flipper.getPIDController(), flipper.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic(){

        flipperPower = flipperController.calculate(flipper.getAbsoluteEncoder(Type.kDutyCycle).getPosition(), intakePosition.position);
        flipperPower = Math.signum(flipperPower) * Math.min(Math.abs(flipperPower), 0.2);
        flipper.set(-flipperPower + feedforward.calculate(.38, .2));
        // flipperController.setReference(0.38, ControlType.kDutyCycle);
        // flipper.set(feedforward.calculate(1, 0));

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

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    public double getFlipperVoltage(){
        return flipper.getBusVoltage();
    }

    public double getFlipPower(){
        return flipperPower;
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