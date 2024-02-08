package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;

public class Cam1Align {
    public static Cam1Align instance;
    private VisionTablesListener vListener;

    private double xPose;
    private double yPose;
    private double zRot;


    //adjust all PID stuff while testing
    private double xSP = -.90;
    private double ySP = -.67;
    private double rotSP = 0;

    //speaker
    private PIDController speakerYController = new PIDController(3.0, 0, 0);
    private PIDController speakerXController = new PIDController(3.0, 0, 0);
    private PIDController speakerRotController = new PIDController(1.2, 0, 0);
    private PIDController[] speakerControllers = {speakerXController, speakerYController, speakerRotController};

    //amp
    private PIDController ampXController = new PIDController(0, 0, 0);
    private PIDController ampYController = new PIDController(0, 0, 0);
    private PIDController ampRotController = new PIDController(0, 0, 0);
    private PIDController[] ampControllers = {ampXController, ampYController, ampRotController};

    //source
    private PIDController sourceXController = new PIDController(0, 0, 0);
    private PIDController sourceYController = new PIDController(0, 0, 0);
    private PIDController sourceRotController = new PIDController(0, 0, 0);
    private PIDController[] sourceControllers = {sourceXController, sourceYController, sourceRotController};

    //stage
    private PIDController stageXController = new PIDController(0, 0, 0);
    private PIDController stageYController = new PIDController(0, 0, 0);
    private PIDController stageRotController = new PIDController(0, 0, 0);
    private PIDController[] stageControllers = {stageXController, stageYController, stageRotController};

    public Cam1Align() {
        vListener = VisionTablesListener.getInstance();
    }

    public static Cam1Align getInstance() {
        if(instance == null) {
            instance = new Cam1Align();
        }
        return instance;
    }

    public PIDController[] getControllers() {
        int id = (int)vListener.getBestID();
        
        if(id == 3 || id == 4 || id == 7 || id == 8) {
            return speakerControllers;
        } else if (id == 5 || id == 6) {
            return ampControllers;
        } else if(id == 1 || id == 2 || id == 9 || id == 10) {
            return sourceControllers;
        } else {
            return stageControllers;
        }
    }

    public double getYSpeed(PIDController controller){
        yPose = vListener.getY();
        PIDController yPoseController = getControllers()[0];

        if(!reachYPose(0.05, yPose)) {
            return yPoseController.calculate(yPose, ySP);
        } 
        return 0;
    }

    public double getXSpeed(PIDController controller){
        xPose = vListener.getX();
        PIDController xPoseController = getControllers()[1];

        if(!reachYPose(0.05, xPose)) {
            return xPoseController.calculate(xPose, xSP);
        }
        return 0;

    }
    public double getRotSpeed(PIDController controller){
        zRot = vListener.getRot();
        PIDController rotController = getControllers()[2];

        if(!reachYPose(0.05, zRot)) {
            return rotController. calculate(zRot, rotSP);
        }
        return 0;
    }

    public boolean reachXPose(double tolerance, double measurment){
        if (Math.abs(measurment - xSP) < tolerance){
            return true;
        }
        return false;
    }

    public boolean reachYPose(double tolerance, double measurment){
        if (Math.abs(measurment - ySP) < tolerance){
            return true;
        }
        return false;
    }

    public boolean reachRot(double tolerance, double measurment){
        if (Math.abs(measurment - rotSP) < tolerance){
            return true;
        }
        return false;
    }
}
