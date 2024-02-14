package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;

public class Cam1Align {
    public static Cam1Align instance;
    private VisionTablesListener vListener;

    private double xPose;
    private double yPose;
    private double zRot;
    private double[] bestTagAbsPos;


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
    
    

    public void getBestTagAbsPos() {
        int id = (int)vListener.getBestID();
        //return format: [x, y, z, rot (degrees)]
        // ID X     Y    Z     Rotation
        // 1 593.68 9.68 53.38 120°
        // 2 637.21 34.79 53.38 120°
        // 3 652.73 196.17 57.13 180°
        // 4 652.73 218.42 57.13 180°
        // 5 578.77 323.00 53.38 270°
        // 6 72.5 323.00 53.38 270°
        // 7 -1.50 218.42 57.13 0°
        // 8 -1.50 196.17 57.13 0°
        // 9 14.02 34.79 53.38 60°
        // 10 57.54 9.68 53.38 60°
        // 11 468.69 146.19 52.00 300°
        // 12 468.69 177.10 52.00 60°
        // 13 441.74 161.62 52.00 180°
        // 14 209.48 161.62 52.00 0°
        // 15 182.73 177.10 52.00 120°
        // 16 182.73 146.19 52.00 240°
        switch(id) {
            case 1:
                bestTagAbsPos = new double[]{593.68, 9.68, 53.38, 120};
                break;
            case 2:
                bestTagAbsPos = new double[]{637.21, 34.79, 53.38, 120};
                break;
            case 3:
                bestTagAbsPos= new double[]{652.73, 196.17, 57.13, 180};
                break;
            case 4:
                bestTagAbsPos= new double[]{652.73, 218.42, 57.13, 180};
                break;
            case 5:
                bestTagAbsPos= new double[]{578.77, 323.00, 53.38, 270};
                break;
            case 6:
                bestTagAbsPos= new double[]{72.5, 323.00, 53.38, 270};
                break;
            case 7:
                bestTagAbsPos= new double[]{-1.50, 218.42, 57.13, 0};
                break;
            case 8:
                bestTagAbsPos= new double[]{-1.50, 196.17, 57.13, 0};
                break;
            case 9:
                bestTagAbsPos= new double[]{14.02, 34.79, 53.38, 60};
                break;
            case 10:
                bestTagAbsPos= new double[]{57.54, 9.68, 53.38, 60};
                break;
            case 11:
                bestTagAbsPos= new double[]{468.69, 146.19, 52.00, 300};
                break;
            case 12:
                bestTagAbsPos= new double[]{468.69, 177.10, 52.00, 60};
                break;
            case 13:
                bestTagAbsPos= new double[]{441.74, 161.62, 52.00, 180};
                break;
            case 14:
                bestTagAbsPos= new double[]{209.48, 161.62, 52.00, 0};
                break;
            case 15:
                bestTagAbsPos= new double[]{182.73, 177.10, 52.00, 120};
                break;
            case 16:
                bestTagAbsPos= new double[]{182.73, 146.19, 52.00, 240};
                break;
            default:
                bestTagAbsPos = null;
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
