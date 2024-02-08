package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;

public class Intake {

     public enum IntakePosition{
        GROUND(-9.3),
        HANDOFF(-1.0);
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

    private double power = .4;

    private double flip = 0.25;

    private ArmFeedforward feedforward;
    private SparkMaxPIDController flipperController;
    private PIDController dumbController;

    private AbsoluteEncoder encoder;
    private RelativeEncoder relativeEncoder;

     private double veloSP = .02;
    
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
        encoder.setVelocityConversionFactor(25);

        encoder.setInverted(true);

        feedforward = new ArmFeedforward(0.0125, 0.0345, 0, 0);
        //low bound: .022 upper bound:.047

        dumbController = new PIDController(IntakeConstants.flipperPCoefficient, IntakeConstants.flipperICoefficient, IntakeConstants.flipperDCoefficient);

        relativeEncoder = flipper.getEncoder();

        flipperController = flipper.getPIDController();
        flipperController.setFeedbackDevice(relativeEncoder);
        flipperController.setOutputRange(-0.25, 0.25);

        flipperController.setP(IntakeConstants.flipperPCoefficient);
        flipperController.setI(IntakeConstants.flipperICoefficient);
        flipperController.setD(IntakeConstants.flipperDCoefficient);

        // control = IntakePID.getInstance(flipper.getPIDController(), flipper.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic(){

        // double reqPower = dumbController.calculate(encoder.getPosition(), intakePosition.position);
        // reqPower = Math.signum(reqPower) * Math.min(Math.abs(reqPower), .15);

        // flipper.set(-reqPower + feedforward.calculate(intakePosition.position, veloSP));

        flipperController.setReference(intakePosition.position, ControlType.kPosition, 0,
         feedforward.calculate(intakePosition.position, veloSP));
        //.31 works

        }

    public void reverseFlipper(){
        flipper.set(-flip + feedforward.calculate(relativeEncoder.getPosition(), veloSP));
    }

    public void setRollerPower(){
        roller.set(power);
    }

    public void setReverseRollerPower(){
        roller.set(-power);
    }

    public void setFlipperPower(){
        flipper.set(flip + feedforward.calculate(relativeEncoder.getPosition(), veloSP));
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

    public double getFlipperPosition(){
        return relativeEncoder.getPosition();
    }

    public double getFlipperVelocity(){
        return encoder.getVelocity();
    }

    public double getFlipperVelocitySetpoint(){
        return veloSP;
    }

    public String getIntakeState(){
        return intakePosition.toString();
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