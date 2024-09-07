package frc.robot.subsystems.launcher;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IO.DigitalInputs;
//import frc.robot.subsystems.vision.VisionTablesListener;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherState {
        // AMP(-48.5, 1.0),
        AMP(-60.5, 1.0),
        ALTAMP(-55, 0.9),
        START(0, 0.0),
        TRAP(-70.04991149902344, 0.8),
        LONG(-15.75, 1.0),
        HANDOFF(9, 0.5),
        HOVER(-3, 1.0),
        TOSS(-22, .80),
        AUTOMIDSHOT(-12, 1.0),
        // height: ?
        AUTOLEFTSHOT(-13.5, 1.0),
        // height: 20.75    
        AUTORIGHTSHOT(-13.5, 1.0),
        // height: ?7
        SPEAKER(-55.5, 1.0),
        ALTSPEAKER(-23, 1.0),
        INTERLOPE(0.0, 1.0),
        TEST(-13.25, 1.0),
        FIXER(-20, 0);

        public double position;
        public double launchSpeed;

        private LauncherState(double position, double launchSpeed) {
            this.position = position;
            this.launchSpeed = launchSpeed;
        }
    }

    public enum LeBronTeam {
        CAVS(-0.25),
        LAKERS(-20);

        public double position;

        private LeBronTeam(double position) {
            this.position = position;
        }
    }

    double anglePower = 0.2;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private CANSparkMax lebronMotor;

    private double increment = 1.0;

    private ArmFeedforward feedForward;
    private ArmFeedforward lebronFeedForward;

    private SparkMaxPIDController pivotController1;

    private SparkMaxPIDController lebronController;

    private static RelativeEncoder encoder;

    private static RelativeEncoder boxScore;

    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[8];

    private static LauncherState launchState = LauncherState.START;
    private static LeBronTeam leBronTeam = LeBronTeam.CAVS;

    public static Launcher instance;

    //public VisionTablesListener visTables;

    private HashMap<Double, Double> lookupTable = new HashMap<>();
    private double[] bluePositions = new double[] { 1.62, 1.93, 2.34, 2.41, 2.63, 2.71, 2.94, 3.01, 3.3, 4.14 };

    public Launcher() {
        shootMotor1 = new CANSparkMax(Ports.shootMotor1, MotorType.kBrushless);
        shootMotor1.restoreFactoryDefaults();

        shootMotor1.setSmartCurrentLimit(60);
        shootMotor1.setIdleMode(IdleMode.kBrake);
        shootMotor1.setInverted(false);
        shootMotor1.burnFlash();

        shootMotor2 = new CANSparkMax(Ports.shootMotor2, MotorType.kBrushless);
        shootMotor2.restoreFactoryDefaults();

        shootMotor2.setSmartCurrentLimit(60);
        shootMotor2.setIdleMode(IdleMode.kBrake);
        shootMotor2.setInverted(false);
        shootMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor = new CANSparkMax(Ports.pivotMotor, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setSmartCurrentLimit(60);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);
        // pivotMotor.setInverted(false);

        pivotMotor.setOpenLoopRampRate(1);

        lebronMotor = new CANSparkMax(Ports.lebron, MotorType.kBrushless);
        lebronMotor.restoreFactoryDefaults();

        lebronMotor.setSmartCurrentLimit(20);
        lebronMotor.setIdleMode(IdleMode.kBrake);

        feedForward = new ArmFeedforward(0.012, 0.017, 0.0, 0.0);
        // u:.023 l:.011 mid:.017 ks:.012

        lebronFeedForward = new ArmFeedforward(0, 0, 0);

        encoder = pivotMotor.getEncoder();

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(encoder);

        pivotController1.setOutputRange(-1, 1);

        lebronController = lebronMotor.getPIDController();

        boxScore = lebronMotor.getEncoder();
        boxScore.setPositionConversionFactor(1);

        lebronController.setFeedbackDevice(boxScore);

        lebronController.setOutputRange(-1, 1);

        lebronController.setP(LauncherConstants.lebronPCoefficient);
        lebronController.setI(LauncherConstants.lebronICoefficient);
        lebronController.setD(LauncherConstants.lebronDCoefficient);

        pivotMotor.burnFlash();
        lebronMotor.burnFlash();

        breakBeam = DigitalInputs.getInstance();

        //visTables = VisionTablesListener.getInstance();

        lookupTable.put(2.94, -10.25);
        lookupTable.put(2.41, -13.25);
        lookupTable.put(2.71, -8.25);
        lookupTable.put(1.62, -18.25);
        lookupTable.put(2.63, -8.25);
        lookupTable.put(2.34, -13.25);
        lookupTable.put(3.01, -10.25);
        lookupTable.put(3.3, -7.5);
        lookupTable.put(1.93, -16.25);
        lookupTable.put(4.14, -7.25);

    }

    public void updatePose() {

        pivotController1.setReference(launchState.position, CANSparkMax.ControlType.kPosition, 0,
                feedForward.calculate(encoder.getPosition(), 0));

        // lebronMotor.set(lebronFeedForward.calculate(boxScore.getPosition(), 0));

        // pivotController1.setReference(15, CANSparkMax.ControlType.kPosition, 0,
        // feedForward.calculate(encoder.getPosition(), 0));

        // pivotController1.setReference(20, ControlType.kPosition, 0,
        // feedForward.calculate(absEncoder.getPosition()* (2* Math.PI) + .349, 0));

        // pivotMotor.set(feedForward.calculate(absEncoder.getPosition()* (2* Math.PI) +
        // .349, 0, 0));

    }

    public void moveLeBron() {
        lebronController.setReference(leBronTeam.position, CANSparkMax.ControlType.kPosition, 0,
                lebronFeedForward.calculate(boxScore.getPosition(), 0));
    }

    public void interpolateAngle() {
        double deltaX;

    //     if (visTables.getLauncherDetects()) {
    //         if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //             deltaX = visTables.getVisX() - .08;
    //         } else {
    //             deltaX = 16.579342 - visTables.getVisX() + .2;
    //         }

    //         double position;

    //         if (deltaX > 1) {
    //             position = -16.1878 * Math.pow(Math.atan(2 / deltaX), 11 / 8);
    //         } else {
    //             position = -20.1878 * Math.pow(Math.atan(2 / deltaX), 11 / 8);
    //         }

    //         LauncherState.INTERLOPE.position = MathUtil.clamp(position, LauncherState.SPEAKER.position,
    //                 LauncherState.HOVER.position);

    //         SmartDashboard.putNumber("DeltaX", deltaX);

    //         // setLauncherState(LauncherState.INTERLOPE);
    //     }
    // }

    // public void lookUpPosition() {
    //     double deltaX;

    //     if (visTables.getLauncherDetects()) {
    //         if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //             deltaX = visTables.getVisX() - 0.08;
    //         } else {
    //             deltaX = 16.579342 - visTables.getVisX() + .2;
    //         }
    //         double position;

    //         if (lookupTable.containsKey(deltaX)) {
    //             position = lookupTable.get(deltaX);
    //         } else {
    //             int upper = 0;
    //             int lower = 0;
    //             for (int i = 0; i < bluePositions.length - 1; i++) {
    //                 if (deltaX < bluePositions[0]) {
    //                     deltaX = LauncherState.ALTSPEAKER.position;
    //                     break;
    //                 } else if (deltaX > bluePositions[bluePositions.length - 1]) {
    //                     deltaX = LauncherState.HOVER.position;
    //                 } else if (deltaX > bluePositions[upper]) {
    //                     lower = upper;
    //                     upper = i + 1;
    //                 }
    //             }

    //             position = deltaX * ((bluePositions[upper] - bluePositions[lower]) / upper - lower);

    //             // SmartDashboard.putNumber("deltaX", deltaX + .08);
    //             // SmartDashboard.putNumber("deltaX", 16.579342 + visTables.getVisX() - .2);
    //             LauncherState.INTERLOPE.position = MathUtil.clamp(position, LauncherState.SPEAKER.position,
    //                     LauncherState.HOVER.position);
    //             // setLauncherState(LauncherState.INTERLOPE);
    //         }
    //     }
     }

    public void eject() {
        shootMotor2.set(0);
        shootMotor1.set(launchState.launchSpeed);
    }

    public void setLeBronOn() {
        lebronMotor.set(-0.5);
    }

    public void setLeBronReverse() {
        lebronMotor.set(0.5);
    }

    public void setLeBronOff() {
        lebronMotor.set(0.0);
    }

    public void setPivotOff() {
        pivotMotor.set(0.0);
    }

    public double getTestPosition() {
        return LauncherState.TEST.position;
    }

     public double getSpeakerPosition() {
        return LauncherState.SPEAKER.position;
    }

    public double getLeBronPostion() {
        return boxScore.getPosition();
    }

    public double getLebronCurrent(){
        return lebronMotor.getOutputCurrent();

    }

    

    

    public void setLauncherOn() {
        if (launchState == LauncherState.AMP) {
            // shootMotor1.set(-launchState.launchSpeed);
            // shootMotor2.set(launchState.launchSpeed * 0.1);
            shootMotor1.set(launchState.launchSpeed * 0.1);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } else if (launchState == LauncherState.ALTAMP) {
            shootMotor1.set(-launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed * 0.1);
        } else {
            shootMotor1.set(launchState.launchSpeed);
            shootMotor2.set(launchState.launchSpeed);
        }
    }

    public void setReverseLauncherOn() {

        // if(launchState == LauncherState.AMP){
        // shootMotor1.set(-launchState.launchSpeed);
        // shootMotor2.set(launchState.launchSpeed * 0.36);
        // } else {
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
        // }
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(1.0);
    }

    public void setFlickerReverse() {
        flicker.set(-1.0);
    }

    public void 
    setFlickerPartial() {
        flicker.set(0.85);
    }

    public void setFlickOff() {
        flicker.set(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean getBreakBeam() {
        return !breakBeam.getInputs()[Ports.launcherBreakBeam];
    }

    public LauncherState getLaunchState() {
        return launchState;
    }

    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - launchState.position) < tolerance;
    }

    public void setLauncherState(LauncherState state) {
        launchState = state;
    }

    public void setLeBronTeam(LeBronTeam team) {
        leBronTeam = team;
    }

    public void increaseIncrement() {
        increment += 0.25;
    }

    public void decreaseInrement() {
        increment -= 0.25;
    }

    public void increasePosition() {
        
        if (launchState == LauncherState.SPEAKER) {
            LauncherState.SPEAKER.position = LauncherState.SPEAKER.position + increment;
        } 

    }

    public void decreasePosition() {
        if (launchState == LauncherState.SPEAKER) {
        LauncherState.SPEAKER.position = LauncherState.SPEAKER.position - increment;
        } 

    }

    public boolean[] launcherConnections() {

        if (shootMotor1.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (shootMotor1.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (shootMotor2.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (shootMotor2.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        if (pivotMotor.getBusVoltage() != 0) {
            connections[4] = true;
        } else {
            connections[4] = false;
        }

        if (pivotMotor.getOutputCurrent() != 0) {
            connections[5] = true;
        } else {
            connections[5] = false;
        }

        if (flicker.getBusVoltage() != 0) {
            connections[6] = true;
        } else {
            connections[6] = false;
        }

        if (flicker.getOutputCurrent() != 0) {
            connections[7] = true;
        } else {
            connections[7] = false;
        }

        return connections;
    }

    public boolean hasBrownedOut() {
        return pivotMotor.getFault(FaultID.kBrownout);
    }

    public void printConnections() {
        SmartDashboard.putBoolean("shootMotor1 Voltage", connections[0]);
        SmartDashboard.putBoolean("shootMotor1 Current", connections[1]);

        SmartDashboard.putBoolean("shootMotor2 Voltage", connections[2]);
        SmartDashboard.putBoolean("shootMotor2 Current", connections[3]);

        SmartDashboard.putBoolean("Pivot Voltage", connections[4]);
        SmartDashboard.putBoolean("Pivot Current", connections[5]);

        SmartDashboard.putBoolean("Flicker Voltage", connections[6]);
        SmartDashboard.putBoolean("Flicker Current", connections[7]);
    }

    public static Launcher getInstance() {
        if (instance == null)
            instance = new Launcher();
        return instance;
    }
}