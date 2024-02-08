package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;

public class Cam2Align {
    private VisionTablesListener vListener;
    public static Cam2Align instance;
    private double xPose;
    private double yPose;

    //adjust below
    private double xSP = -.90;
    private double ySP = -.67;

    private PIDController yPoseController = new PIDController(3.0, 0, 0);
    private PIDController xPoseController = new PIDController(3.0, 0, 0);

    public Cam2Align() {
        vListener = VisionTablesListener.getInstance();
    }

    public static Cam2Align getInstance() {
        if(instance == null)
            instance = new Cam2Align();
        return instance;
    }

    public double getXSpeed() {
        xPose = vListener.getRingX();

        if(!reachYPose(0.05, xPose)) {
            return xPoseController.calculate(xPose, xSP);
        }
        return 0;
    }

    public double getYSpeed() {
        yPose = vListener.getRingY();

        if(!reachYPose(0.05, yPose)) {
            return yPoseController.calculate(yPose, ySP);
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
}
