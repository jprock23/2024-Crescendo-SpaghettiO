package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherPosition{
        AMP(22.857, 23.857),
        HANDOFF(1.066666448116302, 1.32857152223587),
        SPEAKER(35.309, 36.309),
        TEST(6.119045257568359, 5.928566455841064);
        
        public double onePosition;
        public double twoPosition;

        private LauncherPosition(double leftPosition, double twoPosition){
            this.onePosition = leftPosition;
            this.twoPosition = twoPosition;
        }
     }
    

    double power = 1.0;

    double anglePower = 0.25;
    double veloSP = .1;

    private CANSparkMax launchMotor1;
    private CANSparkMax launchMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;

    private ArmFeedforward pivotFeedforward;
    private SparkMaxPIDController pivotController1;
    private SparkMaxPIDController pivotController2;

    private static RelativeEncoder relativeEncoder1;
    private static RelativeEncoder relativeEncoder2;

    private static LauncherPosition launcherPosition = LauncherPosition.AMP;

    // private static LauncherVoltage launcherVolts = LauncherVoltage.OFF;
    // private static FlickerState flickerState = FlickerState.IN;

    public static Launcher instance;
    public static Intake intake;

    private double reqPower1;

    public Launcher() {
        launchMotor1 = new CANSparkMax(Ports.flywheel1, MotorType.kBrushless);
        launchMotor1.restoreFactoryDefaults();

        launchMotor1.setSmartCurrentLimit(60);
        launchMotor1.setIdleMode(IdleMode.kBrake);
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

        pivotController1 = pivotMotor1.getPIDController();
        pivotController2 = pivotMotor2.getPIDController();
        pivotFeedforward = new ArmFeedforward(0.0085, .037, 0.01, 0.0);
        //upper: .045 lower:

        relativeEncoder1 = pivotMotor1.getEncoder();
        relativeEncoder2 = pivotMotor2.getEncoder();
        
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

        intake = Intake.getInstance();

        // control = new LauncherPID(launchMotor1.getPIDController(), launchMotor2.getPIDController(), launchMotor1.getEncoder(), launchMotor2.getEncoder(), 
        // bigFlipper1.getPIDController(), bigFlipper2.getPIDController(), bigFlipper1.getAbsoluteEncoder(Type.kDutyCycle), bigFlipper2.getAbsoluteEncoder(Type.kDutyCycle),
        //  flicker.getPIDController(), flicker.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic(){

        // var desiredState = motionProfile.calculate(0, 
        // new State(relativeEncoder1.getPosition(),  relativeEncoder1.getVelocity()),
        //  new State(launcherPosition.onePosition, 0.0));

        // if(Intake.intakePosition == IntakePosition.HANDOFF){
        //     if (intake.getTimeElapsed() > .32 && intake.hasReachedPose(.36)){
        //         setReverse();
        //         pivotController1.setReference(launcherPosition.onePosition, ControlType.kPosition, 0, pivotFeedforward.calculate(launcherPosition.onePosition, veloSP));
        //         pivotController2.setReference(launcherPosition.twoPosition, ControlType.kPosition, 0, pivotFeedforward.calculate(launcherPosition.twoPosition, veloSP));
        //     }
        // } else {
            pivotController1.setReference(launcherPosition.onePosition,ControlType.kPosition, 0, pivotFeedforward.calculate(launcherPosition.onePosition, veloSP));
            pivotController2.setReference(launcherPosition.twoPosition, ControlType.kPosition, 0, pivotFeedforward.calculate(launcherPosition.twoPosition, veloSP));
    }

    public double getReqPower1(){
        return reqPower1;
    }

    public void setLauncherAngle(){
        pivotMotor1.set(anglePower + pivotFeedforward.calculate(relativeEncoder1.getPosition(), veloSP));
        pivotMotor2.set(anglePower - pivotFeedforward.calculate(relativeEncoder2.getPosition(), veloSP));
    }
    
    public void setReverseLauncherAngle(){
        pivotMotor1.set(-anglePower + pivotFeedforward.calculate(relativeEncoder1.getPosition(), veloSP));
        pivotMotor2.set(-anglePower + pivotFeedforward.calculate(relativeEncoder2.getPosition(), veloSP));

    }

    public void setReverse(){
        launchMotor1.set(-power + pivotFeedforward.calculate(relativeEncoder1.getPosition(), veloSP));
        launchMotor2.set(-power + pivotFeedforward.calculate(relativeEncoder2.getPosition(), veloSP));
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
        return relativeEncoder1.getPosition();

    }

    public double getPosition2(){
        return relativeEncoder2.getPosition();

    }

    public double getPower(){
        return power;
    }

    public String getPivotPosition(){
        return launcherPosition.toString();
    }

    public static double getLauncherPosition1() {
        // return (turnEncoder1.getPosition() + turnEncoder2.getPosition())/2;
        return relativeEncoder1.getPosition();
    }

       public static double getLauncherPosition2() {
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

    public static boolean hasReachedPose(double tolerance) {
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
