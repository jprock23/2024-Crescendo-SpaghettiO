package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;
    final IntegerArraySubscriber tagIDSub;
    private IntegerArraySubscriber xCoordsSub;
    private IntegerArraySubscriber yCoordsSub;
    private IntegerArraySubscriber zRotsSub;
    private IntegerSubscriber bestIDSub;
    private IntegerArraySubscriber ringCenterXSub;
    private IntegerArraySubscriber ringCenterYSub;
    private StringSubscriber cam1StreamSub;
    private double yPose = 0;
    private double xPose = 0;
    private double zRot = 0;
    private double ringX = -1;
    private double ringY = -1;
    private double bestTagID = -1;
    private String cam1Stream = null;
    // private IntegerArraySubscriber xEulerSub;
    // private IntegerArraySubscriber yEulerSub;
    // private IntegerArraySubscriber zEulerSub;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tagIDSub = visionTable.getIntegerArrayTopic("IDs").subscribe(new long[] {});
        xCoordsSub = visionTable.getIntegerArrayTopic("X Coords").subscribe(new long[] {});
        yCoordsSub = visionTable.getIntegerArrayTopic("Y Coords").subscribe(new long[] {});
        zRotsSub = visionTable.getIntegerArrayTopic("Z Euler Angles").subscribe(new long[] {});
        ringCenterXSub = visionTable.getIntegerArrayTopic("Ring Center X Coords").subscribe(new long[] {});
        ringCenterYSub = visionTable.getIntegerArrayTopic("Ring Center Y Coords").subscribe(new long[] {});
        cam1StreamSub = visionTable.getStringTopic("Cam1 Stream").subscribe(new String());
        bestIDSub = visionTable.getIntegerTopic("Best Tag ID").subscribe(-1);
              // xEulerSub = visionTable.getIntegerArrayTopic("X Euler Angles").subscribe(new
        // long[] {});
        // yEulerSub = visionTable.getIntegerArrayTopic("Y Euler Angles").subscribe(new
        // long[] {});
        // // zEulerSub = visionTable.getIntegerArrayTopic("Z Euler
        // Angles").subscribe(new long[] {});
    }

    public void putInfoOnDashboard() {
        boolean tagVisible;
        
        double ringCenterX[];
        double ringCenterY[];
        double[] xPoses;
        double[] yPoses;
        double[] zRots;

        if(tagIDSub.get().length != 0){
            yPoses = convertArray(yCoordsSub.get());
            xPoses = convertArray(xCoordsSub.get());
            zRots = convertArray(zRotsSub.get());
            tagVisible = true;
        } else {
            xPoses = new double[]{-.90}; 
            yPoses = new double[]{-.67}; 
            zRots = new double[]{.5}; 
            tagVisible = false;
        }
        bestTagID = bestIDSub.get();
        SmartDashboard.putNumber("IDs", bestTagID);
        if(xPoses.length != 0){
            xPose = xPoses[0];
            yPose = yPoses[0];
            zRot = zRots[0];   
        
            // SmartDashboard.putNumberArray("IDs", convertArray(tagIDSub.get()));
            SmartDashboard.putNumber("X Coords", xPose);
            SmartDashboard.putNumber("Y Coords", yPose);
            SmartDashboard.putNumber("Z Rot", zRot);
            SmartDashboard.putBoolean("Tag in Sight", tagVisible);
            SmartDashboard.putNumber("Best Tag ID", bestTagID);
            // SmartDashboard.putNumberArray("X Euler Angles",
            // convertArray(xEulerSub.get()));
            // SmartDashboard.putNumberArray("Y Euler Angles",
            // convertArray(yEulerSub.get()));
            // SmartDashboard.putNumberArray("Z Euler Angles",
            // convertArray(zEulerSub.get()));
        }
        
        if(ringCenterXSub.get().length != 0) {
            ringCenterX = convertArray(ringCenterXSub.get());
            ringCenterY = convertArray(ringCenterYSub.get());
        }
        else {
            ringCenterX = new double[]{-1};
            ringCenterY = new double[]{-1};
        }
        
        ringX = ringCenterX[0];
        ringY = ringCenterY[0];
        SmartDashboard.putNumber("Ring X Coord", ringX);
        SmartDashboard.putNumber("Ring Y Coords", ringY);
        
        cam1Stream = cam1StreamSub.get();
        if(cam1Stream != null)
            SmartDashboard.putString("Cam1 Stream", cam1Stream);
    }
    

    // need to convert each value to double individually, can't typecast entire array
    private double[] convertArray(long[] arr) {
        double[] newArr = new double[arr.length];

        for (int i = 0; i < arr.length; i++)
            newArr[i] = (double) (arr[i]) / 1000.0;

        return newArr;
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }

    public double getRot(){
        return zRot;
    }

    public double getY() {
        return yPose;
    }

      public double getX() {
        return xPose;
    }

    public double getRingX() {
        return ringX;
    }

    public double getRingY() {
        return ringY;
    }

    public double getBestID() {
        return bestTagID;
    }
}
