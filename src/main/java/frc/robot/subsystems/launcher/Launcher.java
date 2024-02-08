package frc.robot.subsystems.launcher;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherPosition{
        AMP(22.857, 23.857),
        HANDOFF(0.0, 0.0),
        SPEAKER(35.309, 36.309),
        TESTMID(20.4, 20.4),
        TESTUP(35.309, 36.309),
        TESTDOWN(.738, 1.857);

        public double onePosition;
        public double twoPosition;

        private LauncherPosition(double leftPosition, double twoPosition){
            this.onePosition = leftPosition;
            this.twoPosition = twoPosition;

        }
     }
    

    double power = 1.0;

    double anglePower = 0.35;
    double veloSP = .1;

    private CANSparkMax launchMotor1;
    private CANSparkMax launchMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;

    // private LauncherPID control;

    private AbsoluteEncoder turnEncoder1;
    private AbsoluteEncoder turnEncoder2;

    private ArmFeedforward pivotFeedforward;
    private SparkMaxPIDController pivotController1;
    private SparkMaxPIDController pivotController2;

    private RelativeEncoder relativeEncoder1;
    private RelativeEncoder relativeEncoder2;

    private LauncherPosition launcherPosition = LauncherPosition.HANDOFF;

    private TrapezoidProfile motionProfile;

    // private static LauncherState launcherState = LauncherState.RETRACTED;
    // private static LauncherVoltage launcherVolts = LauncherVoltage.OFF;
    // private static FlickerState flickerState = FlickerState.IN;

    public static Launcher instance;

    private double reqPower1;

    public Launcher() {
        launchMotor1 = new CANSparkMax(Ports.flywheel1, MotorType.kBrushless);
        launchMotor1.restoreFactoryDefaults();

        launchMotor1.setSmartCurrentLimit(60);
        launchMotor1.setIdleMode(IdleMode.kCoast);
        launchMotor1.setInverted(false);
        launchMotor1.burnFlash();

        launchMotor2 = new CANSparkMax(Ports.flywheel2, MotorType.kBrushless);
        launchMotor2.restoreFactoryDefaults();

        launchMotor2.setSmartCurrentLimit(60);
        launchMotor2.setIdleMode(IdleMode.kCoast);
        launchMotor2.setInverted(true);
        launchMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor1 = new CANSparkMax(Ports.bigFlipper1, MotorType.kBrushless);
        pivotMotor1.restoreFactoryDefaults();

        pivotMotor1.setSmartCurrentLimit(60);
        pivotMotor1.setIdleMode(IdleMode.kBrake);
        pivotMotor1.setInverted(false);
        pivotMotor1.setOpenLoopRampRate(1);
        pivotMotor1.burnFlash();

        pivotMotor2 = new CANSparkMax(Ports.bigFlipper2, MotorType.kBrushless);
        pivotMotor2.restoreFactoryDefaults();

        pivotMotor2.setIdleMode(IdleMode.kBrake);
        pivotMotor2.setSmartCurrentLimit(60);
        pivotMotor2.setInverted(true);
        pivotMotor2.setOpenLoopRampRate(1);
        pivotMotor2.burnFlash();

        // veloController = new PIDController(1, 0, 0);
        pivotController1 = pivotMotor1.getPIDController();
        pivotController2 = pivotMotor2.getPIDController();
        // feedforward = new ArmFeedforward(0, 0.25, 0, 0);
        pivotFeedforward = new ArmFeedforward(0.0085, .037, 0.01, 0.0);
        //upper: .045 lower:

        turnEncoder1 = pivotMotor1.getAbsoluteEncoder(Type.kDutyCycle);
        turnEncoder2 = pivotMotor2.getAbsoluteEncoder(Type.kDutyCycle);

        turnEncoder1.setPositionConversionFactor(25);
        turnEncoder2.setPositionConversionFactor(25);
        turnEncoder1.setVelocityConversionFactor(25);
        turnEncoder2.setVelocityConversionFactor(25);

        turnEncoder2.setInverted(true);

        // relativeEncoder1 = pivotMotor1.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        // relativeEncoder2 = pivotMotor2.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);

        relativeEncoder1 = pivotMotor1.getEncoder();
        relativeEncoder2 = pivotMotor2.getEncoder();

        // dumbPivot1 = new PIDController(LauncherConstants.pivotPCoefficient, LauncherConstants.pivotICoefficient, LauncherConstants.pivotDCoefficient);
        // dumbPivot2 = new PIDController(LauncherConstants.pivotPCoefficient, LauncherConstants.pivotICoefficient, LauncherConstants.pivotDCoefficient);
        
        pivotController1 = pivotMotor1.getPIDController();
        pivotController2 = pivotMotor2.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController2.setP(LauncherConstants.pivotPCoefficient);
        pivotController2.setI(LauncherConstants.pivotICoefficient);
        pivotController2.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(relativeEncoder1);
        pivotController2.setFeedbackDevice(relativeEncoder2);

        pivotController1.setOutputRange(-0.6, 0.6);
        pivotController2.setOutputRange(-0.6, 0.6);

        pivotController1.setSmartMotionMaxVelocity(2100, 0);
        pivotController2.setSmartMotionMaxVelocity(2100, 0);
        //theoritcal max: 2100 RPM

        pivotController1.setSmartMotionMaxAccel(5000, 0);
        pivotController2.setSmartMotionMaxAccel(5000, 0);

        motionProfile = new TrapezoidProfile(new Constraints(0.0, 0.0));


        // control = new LauncherPID(launchMotor1.getPIDController(), launchMotor2.getPIDController(), launchMotor1.getEncoder(), launchMotor2.getEncoder(), 
        // bigFlipper1.getPIDController(), bigFlipper2.getPIDController(), bigFlipper1.getAbsoluteEncoder(Type.kDutyCycle), bigFlipper2.getAbsoluteEncoder(Type.kDutyCycle),
        //  flicker.getPIDController(), flicker.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic(){

        var desiredState = motionProfile.calculate(0, 
        new State(relativeEncoder1.getPosition(),  relativeEncoder1.getVelocity()),
         new State(launcherPosition.onePosition, 0.0));


        pivotController1.setReference(launcherPosition.onePosition, ControlType.kPosition, 0, pivotFeedforward.calculate(launcherPosition.onePosition, desiredState.velocity));
        pivotController2.setReference(launcherPosition.twoPosition, ControlType.kPosition, 0, pivotFeedforward.calculate(launcherPosition.twoPosition, desiredState.velocity));
    }

    public double getReqPower1(){
        return reqPower1;
    }

    public void setLauncherAngle(){
        pivotMotor1.set(anglePower + pivotFeedforward.calculate(turnEncoder1.getPosition(), veloSP));
        pivotMotor2.set(anglePower - pivotFeedforward.calculate(turnEncoder1.getPosition(), veloSP));
    }
    
    public void setReverseLauncherAngle(){
        pivotMotor1.set(-anglePower + pivotFeedforward.calculate(turnEncoder1.getPosition(), veloSP));
        pivotMotor2.set(-anglePower + pivotFeedforward.calculate(turnEncoder1.getPosition(), veloSP));

    }

    public void setReverse(){
        launchMotor1.set(-anglePower + pivotFeedforward.calculate(turnEncoder1.getPosition(), veloSP));
        launchMotor2.set(-anglePower/ + pivotFeedforward.calculate(turnEncoder1.getPosition(), veloSP));
    }
    

    public void setAngleStop(){
        pivotMotor1.set(0.0);
        pivotMotor2.set(0.0);
    }

    public void setLauncherPower() {
        launchMotor1.set(power);
        launchMotor2.set(power);
    }

    public void setLaunchZero(){
        launchMotor1.set(0.0);
        launchMotor2.set(0.0);
    }

    public void setFlickerOn(){
        flicker.set(.5);
    }

     public void setFlickerReverse(){
        flicker.set(-.5);
    }

    public void setFlickOff(){
        flicker.set(0);
    }

    public void increasePower(){
        power += .1;
    }

    public void decreasePower(){
        power -= .1;
    }

    public double getPosition(){
        return turnEncoder1.getPosition();

    }

    public double getPosition2(){
        return turnEncoder2.getPosition();

    }

    public double getPower(){
        return power;
    }

    public String getPivotPosition(){
        return launcherPosition.toString();
    }

    public double getLauncherPosition1() {
        // return (turnEncoder1.getPosition() + turnEncoder2.getPosition())/2;
        return relativeEncoder1.getPosition();
    }

       public double getLauncherPosition2() {
        // return (turnEncoder1.getPosition() + turnEncoder2.getPosition())/2;
        return relativeEncoder2.getPosition();
    }

    public double getPivotTargetVelocity(){
        // return (turnEncoder1.getVelocity() + turnEncoder2.getVelocity())/2;
        return pivotController1.getSmartMotionMaxVelocity(0);
    }

    public double getPivotTargetAcceleration() {
        return pivotController1.getSmartMotionMaxAccel(0);
    }

    public double getPivotAcceleration1(double startTime, double endTime){
        double displacement = 34.571;
        double timeDelta = endTime - startTime;
        return (2 * displacement)/Math.pow(timeDelta, 2);
    }

     public double getPivotAcceleration2(double startTime, double endTime){
        double displacement = 34.452;
        double timeDelta = endTime - startTime;
        return (2 * displacement)/Math.pow(timeDelta, 2);
    }

   public double getPivotAcceleration2(){
        return relativeEncoder2.getVelocity();
    }

    public double getPivotVelocity1(){
        return relativeEncoder1.getVelocity();
    }

    public double getPivotVelocity2(){
        return relativeEncoder2.getVelocity();
    }

    public double getPivotVelocitySetPoint(){
        return veloSP;
    }

    public boolean hasReachedPose(double tolerance) {
        if (Math.abs(getLauncherPosition1() - launcherPosition.onePosition) > tolerance) {
            return true;
        }
            return false;
    }

    // public void setFlickerState(FlickerState state){
    //     flickerState = state;
    // }

    public void setPivotState(LauncherPosition state) {
        launcherPosition = state;
    }

    //  public void setLauncherVolts(LauncherVoltage state){
    //     launcherVolts = state;
    // }

    public static Launcher getInstance() {
        if(instance == null)
            instance = new Launcher();
        return instance;
    }
}
