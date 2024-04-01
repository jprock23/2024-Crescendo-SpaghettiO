package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;

    private IntegerArraySubscriber tagIDSub1;
    private DoubleArraySubscriber x1Sub;
    private DoubleArraySubscriber y1Sub;
    private DoubleArraySubscriber z1Sub;
    private DoubleArraySubscriber yaw1Sub;
    private DoubleArraySubscriber timestamp1Sub;

    private IntegerArraySubscriber tagID2Sub;
    private DoubleArraySubscriber x2Sub;
    private DoubleArraySubscriber y2Sub;
    private DoubleArraySubscriber z2Sub;
    private DoubleArraySubscriber yaw2Sub;
    private DoubleArraySubscriber timestamp2Sub;

    private IntegerArraySubscriber tagID3Sub;
    private DoubleArraySubscriber y3Sub;
    private DoubleArraySubscriber x3Sub;
    private DoubleArraySubscriber z3Sub;
    private DoubleArraySubscriber yaw3Sub;
    private DoubleArraySubscriber timestamp3Sub;

    private Transform3d cam1Transform = new Transform3d(new Translation3d(-0.3115, 0.0127, 0.368), new Rotation3d(Math.toRadians(1.5), Math.toRadians(30), Math.toRadians(180)));
    private Transform3d cam2Transform = new Transform3d(new Translation3d(0.2762504, -0.3121152, 0.2306066), new Rotation3d(0, Math.toRadians(35), Math.toRadians(-90)));
    private Transform3d cam3Transform = new Transform3d(new Translation3d(0.2762504, 0.3121152, 0.2306066), new Rotation3d(0, Math.toRadians(35), Math.toRadians(90)));
    // private IntegerArraySubscriber zRotsSub;
    // private IntegerSubscriber bestIDSub;
    // private IntegerSubscriber bestXSub;
    // private IntegerSubscriber bestYSub;
    // private IntegerSubscriber bestZSub;
    // private IntegerArraySubscriber ringCenterXSub;
    // private IntegerArraySubscriber ringCenterYSub;
    // private DoubleSubscriber xTranslationSub;
    // private DoubleSubscriber yTranslationSub;
    // private DoubleSubscriber zTranslationSub;
    // private DoubleSubscriber timeStampSub;

    // private double yPose = 0;
    // private double xPose = 0;
    // private double zRot = 0;
    // private double ringX = -1;
    // private double ringY = -1;
    // private double bestTagID = -1;
    // private double bestTagX = -1;
    // private double bestTagY = -1;
    // private double bestTagZ = -1;

    // private double xTranslation;
    // private double yTranslation;
    // private double zTranslation;

    // private double timestamp;

    private boolean tagVisible;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");

        tagIDSub1 = visionTable.getIntegerArrayTopic("IDs").subscribe(new long[] {});
        x1Sub = visionTable.getDoubleArrayTopic("X Coords").subscribe(new double[] {});
        y1Sub = visionTable.getDoubleArrayTopic("Y Coords").subscribe(new double[] {});
        z1Sub = visionTable.getDoubleArrayTopic("Z Coords").subscribe(new double[] {});
        yaw1Sub = visionTable.getDoubleArrayTopic("Yaws").subscribe(new double[] {});
        timestamp1Sub = visionTable.getDoubleArrayTopic("Timestamps").subscribe(new double[] {});
        
        tagID2Sub = visionTable.getIntegerArrayTopic("IDs 2").subscribe(new long[] {});
        x2Sub = visionTable.getDoubleArrayTopic("X Coords 2").subscribe(new double[] {});
        y2Sub = visionTable.getDoubleArrayTopic("Y Coords 2").subscribe(new double[] {});
        z2Sub = visionTable.getDoubleArrayTopic("Z Coords 2").subscribe(new double[] {});
        yaw2Sub = visionTable.getDoubleArrayTopic("Yaws 2").subscribe(new double[] {});
        timestamp2Sub = visionTable.getDoubleArrayTopic("Timestamps 2").subscribe(new double[] {});

        tagID3Sub = visionTable.getIntegerArrayTopic("IDs 3").subscribe(new long[] {});
        x3Sub = visionTable.getDoubleArrayTopic("X Coords 3").subscribe(new double[] {});
        y3Sub = visionTable.getDoubleArrayTopic("Y Coords 3").subscribe(new double[] {});
        z3Sub = visionTable.getDoubleArrayTopic("Z Coords 3").subscribe(new double[] {});
        yaw3Sub = visionTable.getDoubleArrayTopic("Yaws 3").subscribe(new double[] {});
        timestamp3Sub = visionTable.getDoubleArrayTopic("Timestamps 3").subscribe(new double[] {});

        // zRotsSub = visionTable.getIntegerArrayTopic("Z Euler Angles").subscribe(new long[] {});
        // ringCenterXSub = visionTable.getIntegerArrayTopic("Ring Center X Coords").subscribe(new long[] {});
        // ringCenterYSub = visionTable.getIntegerArrayTopic("Ring Center Y Coords").subscribe(new long[] {});
        // bestIDSub = visionTable.getIntegerTopic("Best Tag ID").subscribe(-1);
        // xTranslationSub = visionTable.getDoubleTopic("Best Tag X").subscribe(-1.0);
        // yTranslationSub = visionTable.getDoubleTopic("Best Tag Y").subscribe(-1.0);
        // zTranslationSub = visionTable.getDoubleTopic("Best Tag Z").subscribe(-1.0);
        // timeStampSub = visionTable.getDoubleTopic("Best Timestamp").subscribe(-1.0);
    }

    public void putInfoOnDashboard() {
        if(getTagVisible1() || getTagVisible2() || getTagVisible3()) {
            tagVisible = true;
        }
        else {
            tagVisible = false;
        }

        SmartDashboard.putBoolean("Tag in Sight", tagVisible);
        // double ringCenterX[];
        // double ringCenterY[];
        //double[] xPoses;
        //double[] yPoses;
        // double[] zRots;

        // if (tagIDSub.get().length != 0) {
            
        //     yPoses = ySub.get();
        //     xPoses = xSub.get();
        //     // zRots = convertArray(zRotsSub.get());

        //     // xTranslation = xTranslationSub.get();
        //     // yTranslation = yTranslationSub.get();
        //     // zTranslation = zTranslationSub.get();

        //     // timestamp = timeStampSub.get();
        //     tagVisible = true;
        // } else {
        //     xPoses = new double[] { -.90 };
        //     yPoses = new double[] { -.67 };
        //     //zRots = new double[] { .5 };
        //     //tagVisible = false;
        // }
        // bestTagID = bestIDSub.get();
        // bestTagX = bestXSub.get();
        // bestTagY = bestYSub.get();
        // bestTagZ = bestZSub.get();
        //SmartDashboard.putNumber("IDs", bestTagID);
        //if (xPoses.length != 0) {
            //xPose = xPoses[0];
            //yPose = yPoses[0];
            //zRot = zRots[0];

            // SmartDashboard.putNumberArray("IDs", convertArray(tagIDSub.get()));
            // SmartDashboard.putNumber("X Coords", xPose);
            // SmartDashboard.putNumber("Y Coords", yPose);
            // SmartDashboard.putNumber("Z Rot", zRot);
            // SmartDashboard.putBoolean("Tag in Sight", tagVisible);
            // SmartDashboard.putNumber("Best Tag ID", bestTagID);
            // SmartDashboard.putNumberArray("X Euler Angles",
            // convertArray(xEulerSub.get()));
            // SmartDashboard.putNumberArray("Y Euler Angles",
            // convertArray(yEulerSub.get()));
            // SmartDashboard.putNumberArray("Z Euler Angles",
            // convertArray(zEulerSub.get()));
        //}

        // if (ringCenterXSub.get().length != 0) {
        //     ringCenterX = convertArray(ringCenterXSub.get());
        //     ringCenterY = convertArray(ringCenterYSub.get());
        // } else {
        //     ringCenterX = new double[] { -1 };
        //     ringCenterY = new double[] { -1 };
        // }

        // ringX = ringCenterX[0];
        // ringY = ringCenterY[0];
        // SmartDashboard.putNumber("Ring X Coord", ringX);
        // SmartDashboard.putNumber("Ring Y Coords", ringY);

    }

    public boolean getTagVisible1() {
        return tagIDSub1.get().length  > 0;
    }

     public boolean getTagVisible2() {
        return tagID2Sub.get().length  > 0;
    }

    public boolean getTagVisible3() {
        return tagID3Sub.get().length  > 0;
    }


    // need to convert each value to double individually, can't typecast entire array
    private double[] convertArray(long[] arr) {
        double[] newArr = new double[arr.length];

        for (int i = 0; i < arr.length; i++)
            newArr[i] = (double) (arr[i]) / 1.0;

        return newArr;
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }

    public double[] getCam1IDs() {
        return convertArray(tagIDSub1.get());
    }

    public double[] getCam2IDs() {
        return convertArray(tagID2Sub.get());
    }

    public double[] getCam3IDs() {
        return convertArray(tagID3Sub.get());
    }

    public double[] getCam1Timestamps() {
        return timestamp1Sub.get();
    }

    public double[] getCam2Timestamps() {
        return timestamp2Sub.get();
    }

    public double[] getCam3Timestamps() {
        return timestamp3Sub.get();
    }

    // public double getRot() {
    //     return zRot;
    // }

    // public double getY() {
    //     return yPose;
    // }

    // public double getX() {
    //     return xPose;
    // }

    // public double getRingX() {
    //     return ringX;
    // }

    // public double getRingY() {
    //     return ringY;
    // }

    // public double getBestID() {
    //     return bestTagID;
    // }

    // public Translation3d getBestPos() {
    //     return new Translation3d(bestTagX, bestTagY, bestTagZ);
    // }

    public Pose2d[] getCam1Poses() {
        double[] ids = convertArray(tagIDSub1.get());
        double[] xPoses = x1Sub.get();
        Logger.recordOutput("Tag 1 X", xPoses);
        double[] yPoses = y1Sub.get();
        Logger.recordOutput("Tag 1 Y", yPoses);
        double[] zPoses = z1Sub.get();
        Logger.recordOutput("Tag 1 Z", zPoses);
        double[] yaws = yaw1Sub.get();
        Logger.recordOutput("Tag 1 Yaws", yaws);
        
        Pose2d[] poses = new Pose2d[ids.length];
        for(int i = 0; i < ids.length && i < xPoses.length && i < yPoses.length && i < zPoses.length; i++) {
            Translation3d translate = new Translation3d(xPoses[i], yPoses[i], zPoses[i]);
            Rotation3d rotation = new Rotation3d(0, 0, yaws[i]);
            Transform3d transform = new Transform3d(translate, rotation);
            transform = new Transform3d(
                CoordinateSystem.convert(transform.getTranslation(), CoordinateSystem.EDN(), CoordinateSystem.NWU()),
                CoordinateSystem.convert(new Rotation3d(), CoordinateSystem.EDN(), CoordinateSystem.NWU()).plus(
                    CoordinateSystem.convert(transform.getRotation(), CoordinateSystem.EDN(), CoordinateSystem.NWU())
                )
            );
            poses[i] = ComputerVisionUtil.objectToRobotPose(getBestTagAbsPos((int)ids[i]), transform, cam1Transform).toPose2d();
        }
        return poses;
    }

    public Transform3d[] getCam2Transforms() {
        double[] ids = convertArray(tagID2Sub.get());
        double[] xPoses = x2Sub.get();
        Logger.recordOutput("Tag 2 X", xPoses);
        double[] yPoses = y2Sub.get();
        Logger.recordOutput("Tag 2 Y", yPoses);
        double[] zPoses = z2Sub.get();
        Logger.recordOutput("Tag 2 Z", zPoses);
        double[] yaws = yaw2Sub.get();
        Logger.recordOutput("Tag 2 Yaws", yaws);
        
        Transform3d[] transforms = new Transform3d[ids.length];
        for(int i = 0; i < ids.length; i++) {
            Translation3d translate = new Translation3d(xPoses[i], yPoses[i], zPoses[i]);
            Rotation3d rotation = new Rotation3d(0, 0, yaws[i]);
            Transform3d transform = new Transform3d(translate, rotation);

            transform.plus(cam2Transform);
            transforms[i] = transform;
        }
        return transforms;
    }

    public Transform3d[] getCam3Transforms() {
        double[] ids = convertArray(tagID3Sub.get());
        double[] xPoses = x3Sub.get();
        Logger.recordOutput("Tag 3 X", xPoses);
        double[] yPoses = y3Sub.get();
        Logger.recordOutput("Tag 3 Y", yPoses);
        double[] zPoses = z3Sub.get();
        Logger.recordOutput("Tag 3 Z", zPoses);
        double[] yaws = yaw3Sub.get();
        Logger.recordOutput("Tag 3 Yaws", yaws);
        
        Transform3d[] transforms = new Transform3d[ids.length];
        for(int i = 0; i < ids.length; i++) {
            Translation3d translate = new Translation3d(xPoses[i], yPoses[i], zPoses[i]);
            Rotation3d rotation = new Rotation3d(0, 0, yaws[i]);
            Transform3d transform = new Transform3d(translate, rotation);

            transform.plus(cam3Transform);
            transforms[i] = transform;
        }
        return transforms;
    }
    public Pose2d[] getCam1RobotPoses() {
        double[] xPoses = x1Sub.get();
        double[] yPoses = y1Sub.get();
        double[] zPoses = z1Sub.get();
        double[] yaws = yaw1Sub.get();
        ArrayList<Pose3d> poses = new ArrayList<Pose3d>();
        for(int i = 0; i < xPoses.length && i < yPoses.length && i < zPoses.length && i < yaws.length; i++) {
            poses.add(new Pose3d(
                new Translation3d(xPoses[i], yPoses[i], zPoses[i]), 
                new Rotation3d(0, 0, yaws[i])
            ));
        }

        Pose2d[] pose2ds = new Pose2d[poses.size()];
        int i = 0;
        for(Pose3d pose: poses) {
            pose2ds[i] = pose.toPose2d();
            i++;
        }
        return pose2ds;   
    }

    public Pose3d getBestTagAbsPos(int id) {

        // format: [x, y, z, rot]
        // ID X Y Z Rotation
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

        Pose3d bestTagAbsPos;
        switch (id) {
            case 1:
                bestTagAbsPos = new Pose3d(new Translation3d(15.079472, 0.245872, 1.355852), new Rotation3d(0, 0, Math.toRadians(120)));
                break;
            case 2:
                bestTagAbsPos = new Pose3d(new Translation3d(16.185134, 0.883666, 1.355852), new Rotation3d(0, 0, Math.toRadians(120)));
                break;
            case 3:
                bestTagAbsPos = new Pose3d(new Translation3d(16.579342, 4.982718, 1.451102), new Rotation3d(0, 0, Math.toRadians(180)));
                break;
            case 4:
                bestTagAbsPos = new Pose3d(new Translation3d(16.579342, 5.547868, 1.451102), new Rotation3d(0, 0, Math.toRadians(180)));
                break;
            case 5:
                bestTagAbsPos = new Pose3d(new Translation3d(14.700758, 8.2042, 1.355852), new Rotation3d(0, 0, Math.toRadians(270)));
                break;
            case 6:
                bestTagAbsPos = new Pose3d(new Translation3d(1.8415, 8.2042, 1.355852), new Rotation3d(0, 0, Math.toRadians(270)));
                break;
            case 7:
                bestTagAbsPos = new Pose3d(new Translation3d(-0.0381, 5.547868, 1.451102), new Rotation3d(0, 0, Math.toRadians(0)));
                break;
            case 8:
                bestTagAbsPos = new Pose3d(new Translation3d(-0.0381, 4.982718, 1.451102), new Rotation3d(0, 0, Math.toRadians(0)));
                break;
            case 9:
                bestTagAbsPos = new Pose3d(new Translation3d(0.356108, 0.883666, 1.355852), new Rotation3d(0, 0, Math.toRadians(60)));
                break;
            case 10:
                bestTagAbsPos = new Pose3d(new Translation3d(1.461516, 0.245872, 1.355852), new Rotation3d(0, 0, Math.toRadians(60)));
                break;
            case 11:
                bestTagAbsPos = new Pose3d(new Translation3d(11.904726, 3.713226, 1.3208), new Rotation3d(0, 0, Math.toRadians(300)));
                break;
            case 12:
                bestTagAbsPos = new Pose3d(new Translation3d(11.904726, 4.49834, 1.3208), new Rotation3d(0, 0, Math.toRadians(60)));
                break;
            case 13:
                bestTagAbsPos = new Pose3d(new Translation3d(11.220196, 4.105148, 1.3208), new Rotation3d(0, 0, Math.toRadians(180)));
                break;
            case 14:
                bestTagAbsPos = new Pose3d(new Translation3d(5.320792, 4.105148, 1.3208), new Rotation3d(0, 0, Math.toRadians(0)));
                break;
            case 15:
                bestTagAbsPos = new Pose3d(new Translation3d(4.641342, 4.49834, 1.3208), new Rotation3d(0, 0, Math.toRadians(120)));
                break;
            case 16:
                bestTagAbsPos = new Pose3d(new Translation3d(4.641342, 3.713226, 1.3208), new Rotation3d(0, 0, Math.toRadians(240)));
                break;
            default:
                bestTagAbsPos = null;
        }

        return bestTagAbsPos;
    }
}
