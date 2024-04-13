package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;

    private StringArraySubscriber tag1Sub;
    private StringArraySubscriber tag2Sub;
    private StringArraySubscriber tag3Sub;

    // TODO: verify cam1-cam3 measurements (x, y, z, and ccw yay fom camera to
    // center of robot)
    // TODO: If using 3 cameras, write analagous methods to cam1 for cam2 and cam3
    private Transform3d cam1toRobot = new Transform3d(new Translation3d(-0.3115, 0, 0.368),
            new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180)));
    private Transform3d cam2toRobot = new Transform3d(new Translation3d(0.3302, 0.2762504, 0.2306066),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(-90)));
    private Transform3d cam3toRobot = new Transform3d(new Translation3d(0.3302, -0.2762504, 0.2306066),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(90)));

    private static LinkedList<AprilTagDetection> launcherDetections = new LinkedList<AprilTagDetection>();
    private static LinkedList<AprilTagDetection> leftDetections = new LinkedList<AprilTagDetection>();
    private static LinkedList<AprilTagDetection> rightDetections = new LinkedList<AprilTagDetection>();

    private boolean launcherVisible;
    private boolean leftVisible;
    private boolean rightVisible;

    private AprilTagFieldLayout tagLayout;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tag1Sub = visionTable.getStringArrayTopic("Serialized Tags").subscribe(new String[] {});
        tag2Sub = visionTable.getStringArrayTopic("Serialized Tags 2").subscribe(new String[] {});
        tag3Sub = visionTable.getStringArrayTopic("Serialized Tags 3").subscribe(new String[] {});

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource("2024-crescendo.json");
        } catch (IOException e) {

        }
    }

    public void printDetects() {
        printLauncherDetects();
        printLeftDetects();
        printRightDetects();
    }

    public void printLauncherDetects() {
        SmartDashboard.putBoolean("Launcher Tag", launcherVisible);
    }

    public void printLeftDetects() {
        SmartDashboard.putBoolean("Left Tag", leftVisible);
    }

    public void printRightDetects() {
        SmartDashboard.putBoolean("Right Tag", rightVisible);
    }

    public boolean getLauncherDetects() {
        if (tag1Sub.get().length > 0)
            launcherVisible = true;
        else
            launcherVisible = false;

        return launcherVisible;
    }

    public boolean getLeftDetects() {
        if (tag2Sub.get().length > 0)
            leftVisible = true;
        else
            leftVisible = false;

        return leftVisible;
    }

    public boolean getRightDetects() {
        if (tag3Sub.get().length > 0)
            rightVisible = true;
        else
            rightVisible = false;

        return rightVisible;
    }

    public Pose2d[] getLauncherPoses() {
        launcherDetections.clear();
        String[] serializedTags = tag1Sub.get();
        for (String tagSerial : serializedTags)
            launcherDetections.add(new AprilTagDetection(tagSerial));

        if (launcherDetections == null || launcherDetections.size() == 0)
            return null;

        Pose2d[] poses = new Pose2d[launcherDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(launcherDetections.get(i).getID()).get();
            Transform3d robotPos = launcherDetections.get(i).getTransform().plus(cam1toRobot);
            robotPos = new Transform3d(
                    new Translation3d(
                            robotPos.getX(),
                            -robotPos.getY(),
                            -robotPos.getZ()),
                    robotPos.getRotation());
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();
        }
        return poses;
    }

    public double[] getLauncherTimeStamps() {
        double[] launcherTimestamps = new double[launcherDetections.size()];

        for (int i = 0; i < launcherDetections.size(); i++)
            launcherTimestamps[i] = launcherDetections.get(i).getTimeStamp();

        return launcherTimestamps;
    }

    public Pose2d[] getRightPoses() {
        rightDetections.clear();
        String[] serializedTags = tag3Sub.get();
        for (String tagSerial : serializedTags)
            rightDetections.add(new AprilTagDetection(tagSerial));

        if (rightDetections == null || rightDetections.size() == 0)
            return null;

        Pose2d[] poses = new Pose2d[rightDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(rightDetections.get(i).getID()).get();
            Transform3d robotPos = rightDetections.get(i).getTransform().plus(cam3toRobot);
            robotPos = new Transform3d(
                    new Translation3d(
                            robotPos.getX(),
                            -robotPos.getY(),
                            -robotPos.getZ()),
                    robotPos.getRotation());
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();
        }
        return poses;
    }

    public double[] getRightTimeStamps() {
        double[] rightTimeStamps = new double[rightDetections.size()];

        for (int i = 0; i < rightDetections.size(); i++)
            rightTimeStamps[i] = rightDetections.get(i).getTimeStamp();

        return rightTimeStamps;
    }

    public Pose2d[] getLeftPose() {
        leftDetections.clear();
        String[] serializedTags = tag2Sub.get();
        for (String tagSerial : serializedTags)
            leftDetections.add(new AprilTagDetection(tagSerial));

        if (leftDetections == null || leftDetections.size() == 0)
            return null;

        Pose2d[] poses = new Pose2d[rightDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(leftDetections.get(i).getID()).get();
            Transform3d robotPos = leftDetections.get(i).getTransform().plus(cam2toRobot);
            robotPos = new Transform3d(
                    new Translation3d(
                            robotPos.getX(),
                            -robotPos.getY(),
                            -robotPos.getZ()),
                    robotPos.getRotation());
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();
        }
        return poses;
    }

    public double[] getLeftTimeStamps() {
        double[] leftTimeStamps = new double[leftDetections.size()];

        for (int i = 0; i < rightDetections.size(); i++)
            leftTimeStamps[i] = rightDetections.get(i).getTimeStamp();

        return leftTimeStamps;
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }

}
