//LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER)

package example.robot.lib;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.concurrent.ConcurrentHashMap;
import edu.wpi.first.net.PortForwarder;

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
public class LimelightHelpers {

    private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

    public static class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private  double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()  { return toPose3D(cameraPose_TargetSpace); }
        public Pose3d getRobotPose_FieldSpace()    { return toPose3D(robotPose_FieldSpace); }
        public Pose3d getRobotPose_TargetSpace()   { return toPose3D(robotPose_TargetSpace); }
        public Pose3d getTargetPose_CameraSpace()  { return toPose3D(targetPose_CameraSpace); }
        public Pose3d getTargetPose_RobotSpace()   { return toPose3D(targetPose_RobotSpace); }
        public Pose2d getCameraPose_TargetSpace2D(){ return toPose2D(cameraPose_TargetSpace); }
        public Pose2d getRobotPose_FieldSpace2D()  { return toPose2D(robotPose_FieldSpace); }
        public Pose2d getRobotPose_TargetSpace2D() { return toPose2D(robotPose_TargetSpace); }
        public Pose2d getTargetPose_CameraSpace2D(){ return toPose2D(targetPose_CameraSpace); }
        public Pose2d getTargetPose_RobotSpace2D() { return toPose2D(targetPose_RobotSpace); }

        @JsonProperty("ta")  public double ta;
        @JsonProperty("tx")  public double tx;
        @JsonProperty("ty")  public double ty;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("typ") public double ty_pixels;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        @JsonProperty("ts")  public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace   = new double[6];
            robotPose_TargetSpace  = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace  = new double[6];
        }
    }

    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")  public double fiducialID;
        @JsonProperty("fam")  public String fiducialFamily;

        @JsonProperty("t6c_ts") private double[] cameraPose_TargetSpace;
        @JsonProperty("t6r_fs") private double[] robotPose_FieldSpace;
        @JsonProperty("t6r_ts") private double[] robotPose_TargetSpace;
        @JsonProperty("t6t_cs") private double[] targetPose_CameraSpace;
        @JsonProperty("t6t_rs") private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()  { return toPose3D(cameraPose_TargetSpace); }
        public Pose3d getRobotPose_FieldSpace()    { return toPose3D(robotPose_FieldSpace); }
        public Pose3d getRobotPose_TargetSpace()   { return toPose3D(robotPose_TargetSpace); }
        public Pose3d getTargetPose_CameraSpace()  { return toPose3D(targetPose_CameraSpace); }
        public Pose3d getTargetPose_RobotSpace()   { return toPose3D(targetPose_RobotSpace); }
        public Pose2d getCameraPose_TargetSpace2D(){ return toPose2D(cameraPose_TargetSpace); }
        public Pose2d getRobotPose_FieldSpace2D()  { return toPose2D(robotPose_FieldSpace); }
        public Pose2d getRobotPose_TargetSpace2D() { return toPose2D(robotPose_TargetSpace); }
        public Pose2d getTargetPose_CameraSpace2D(){ return toPose2D(targetPose_CameraSpace); }
        public Pose2d getTargetPose_RobotSpace2D() { return toPose2D(targetPose_RobotSpace); }

        @JsonProperty("ta")  public double ta;
        @JsonProperty("tx")  public double tx;
        @JsonProperty("ty")  public double ty;
        @JsonProperty("txp") public double tx_pixels;
        @JsonProperty("typ") public double ty_pixels;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        @JsonProperty("ts")  public double ts;

        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace   = new double[6];
            robotPose_TargetSpace  = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace  = new double[6];
        }
    }

    public static class LimelightTarget_Barcode {
        @JsonProperty("fam")  public String family;
        @JsonProperty("data") public String data;
        @JsonProperty("txp")  public double tx_pixels;
        @JsonProperty("typ")  public double ty_pixels;
        @JsonProperty("tx")   public double tx;
        @JsonProperty("ty")   public double ty;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        @JsonProperty("ta")   public double ta;
        @JsonProperty("pts")  public double[][] corners;
        public LimelightTarget_Barcode() {}
        public String getFamily() { return family; }
    }

    public static class LimelightTarget_Classifier {
        @JsonProperty("class")   public String className;
        @JsonProperty("classID") public double classID;
        @JsonProperty("conf")    public double confidence;
        @JsonProperty("zone")    public double zone;
        @JsonProperty("tx")      public double tx;
        @JsonProperty("txp")     public double tx_pixels;
        @JsonProperty("ty")      public double ty;
        @JsonProperty("typ")     public double ty_pixels;
        public LimelightTarget_Classifier() {}
    }

    public static class LimelightTarget_Detector {
        @JsonProperty("class")   public String className;
        @JsonProperty("classID") public double classID;
        @JsonProperty("conf")    public double confidence;
        @JsonProperty("ta")      public double ta;
        @JsonProperty("tx")      public double tx;
        @JsonProperty("ty")      public double ty;
        @JsonProperty("txp")     public double tx_pixels;
        @JsonProperty("typ")     public double ty_pixels;
        @JsonProperty("tx_nocross") public double tx_nocrosshair;
        @JsonProperty("ty_nocross") public double ty_nocrosshair;
        public LimelightTarget_Detector() {}
    }

    public static class HardwareReport {
        @JsonProperty("cid")   public String cameraId;
        @JsonProperty("cpu")   public double cpuUsage;
        @JsonProperty("dfree") public double diskFree;
        @JsonProperty("dtot")  public double diskTotal;
        @JsonProperty("ram")   public double ramUsage;
        @JsonProperty("temp")  public double temperature;
        public HardwareReport() {}
    }

    public static class IMUResults {
        @JsonProperty("data") public double[] data;
        @JsonProperty("quat") public double[] quaternion;
        @JsonProperty("yaw")  public double yaw;

        public double robotYaw, roll, pitch, rawYaw;
        public double gyroZ, gyroX, gyroY;
        public double accelZ, accelX, accelY;

        public IMUResults() {
            data       = new double[0];
            quaternion = new double[4];
        }

        public void parseDataArray() {
            if (data != null && data.length >= 10) {
                robotYaw = data[0]; roll  = data[1]; pitch = data[2]; rawYaw = data[3];
                gyroZ    = data[4]; gyroX = data[5]; gyroY = data[6];
                accelZ   = data[7]; accelX = data[8]; accelY = data[9];
            }
        }
    }

    public static class RewindStats {
        @JsonProperty("bufferUsage")  public double bufferUsage;
        @JsonProperty("enabled")      public int enabled;
        @JsonProperty("flushing")     public int flushing;
        @JsonProperty("frameCount")   public int frameCount;
        @JsonProperty("latpen")       public int latencyPenalty;
        @JsonProperty("storedSeconds")public double storedSeconds;
        public RewindStats() {}
    }

    public static class LimelightResults {
        public String error;

        @JsonProperty("pID") public double pipelineID;
        @JsonProperty("tl")  public double latency_pipeline;
        @JsonProperty("cl")  public double latency_capture;
        public double latency_jsonParse;

        @JsonProperty("ts")     public double timestamp_LIMELIGHT_publish;
        @JsonProperty("ts_rio") public double timestamp_RIOFPGA_capture;
        @JsonProperty("ts_nt")  public long   timestamp_nt;
        @JsonProperty("ts_sys") public long   timestamp_sys;
        @JsonProperty("ts_us")  public long   timestamp_us;

        @JsonProperty("v") @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("pTYPE") public String pipelineType;
        @JsonProperty("tx")    public double tx;
        @JsonProperty("ty")    public double ty;
        @JsonProperty("txnc")  public double tx_nocrosshair;
        @JsonProperty("tync")  public double ty_nocrosshair;
        @JsonProperty("ta")    public double ta;

        @JsonProperty("botpose")          public double[] botpose;
        @JsonProperty("botpose_wpired")   public double[] botpose_wpired;
        @JsonProperty("botpose_wpiblue")  public double[] botpose_wpiblue;
        @JsonProperty("botpose_tagcount") public double botpose_tagcount;
        @JsonProperty("botpose_span")     public double botpose_span;
        @JsonProperty("botpose_avgdist")  public double botpose_avgdist;
        @JsonProperty("botpose_avgarea")  public double botpose_avgarea;

        @JsonProperty("botpose_orb")         public double[] botpose_orb;
        @JsonProperty("botpose_orb_wpiblue") public double[] botpose_orb_wpiblue;
        @JsonProperty("botpose_orb_wpired")  public double[] botpose_orb_wpired;
        @JsonProperty("t6c_rs")              public double[] camerapose_robotspace;

        @JsonProperty("hw")     public HardwareReport hardware;
        @JsonProperty("imu")    public IMUResults imuResults;
        @JsonProperty("rewind") public RewindStats rewindStats;
        @JsonProperty("PythonOut") public double[] pythonOutput;

        public Pose3d getBotPose3d()         { return toPose3D(botpose); }
        public Pose3d getBotPose3d_wpiRed()  { return toPose3D(botpose_wpired); }
        public Pose3d getBotPose3d_wpiBlue() { return toPose3D(botpose_wpiblue); }
        public Pose2d getBotPose2d()         { return toPose2D(botpose); }
        public Pose2d getBotPose2d_wpiRed()  { return toPose2D(botpose_wpired); }
        public Pose2d getBotPose2d_wpiBlue() { return toPose2D(botpose_wpiblue); }

        @JsonProperty("Retro")      public LimelightTarget_Retro[]      targets_Retro;
        @JsonProperty("Fiducial")   public LimelightTarget_Fiducial[]   targets_Fiducials;
        @JsonProperty("Classifier") public LimelightTarget_Classifier[] targets_Classifier;
        @JsonProperty("Detector")   public LimelightTarget_Detector[]   targets_Detector;
        @JsonProperty("Barcode")    public LimelightTarget_Barcode[]    targets_Barcode;

        public LimelightResults() {
            botpose = new double[6]; botpose_wpired = new double[6]; botpose_wpiblue = new double[6];
            botpose_orb = new double[6]; botpose_orb_wpiblue = new double[6]; botpose_orb_wpired = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro       = new LimelightTarget_Retro[0];
            targets_Fiducials   = new LimelightTarget_Fiducial[0];
            targets_Classifier  = new LimelightTarget_Classifier[0];
            targets_Detector    = new LimelightTarget_Detector[0];
            targets_Barcode     = new LimelightTarget_Barcode[0];
            pythonOutput        = new double[0];
            pipelineType        = "";
        }
    }

    public static class RawFiducial {
        public int    id = 0;
        public double txnc = 0, tync = 0, ta = 0;
        public double distToCamera = 0, distToRobot = 0, ambiguity = 0;

        public RawFiducial(int id, double txnc, double tync, double ta,
                double distToCamera, double distToRobot, double ambiguity) {
            this.id = id; this.txnc = txnc; this.tync = tync; this.ta = ta;
            this.distToCamera = distToCamera; this.distToRobot = distToRobot; this.ambiguity = ambiguity;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            RawFiducial o = (RawFiducial) obj;
            return id == o.id && Double.compare(txnc, o.txnc) == 0 && Double.compare(tync, o.tync) == 0
                && Double.compare(ta, o.ta) == 0 && Double.compare(distToCamera, o.distToCamera) == 0
                && Double.compare(distToRobot, o.distToRobot) == 0 && Double.compare(ambiguity, o.ambiguity) == 0;
        }
    }

    public static class RawTarget {
        public double txnc = 0, tync = 0, ta = 0;
        public RawTarget(double txnc, double tync, double ta) {
            this.txnc = txnc; this.tync = tync; this.ta = ta;
        }
        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            RawTarget o = (RawTarget) obj;
            return Double.compare(txnc, o.txnc) == 0 && Double.compare(tync, o.tync) == 0 && Double.compare(ta, o.ta) == 0;
        }
    }

    public static class RawDetection {
        public int    classId = 0;
        public double txnc = 0, tync = 0, ta = 0;
        public double corner0_X = 0, corner0_Y = 0, corner1_X = 0, corner1_Y = 0;
        public double corner2_X = 0, corner2_Y = 0, corner3_X = 0, corner3_Y = 0;

        public RawDetection(int classId, double txnc, double tync, double ta,
                double c0x, double c0y, double c1x, double c1y,
                double c2x, double c2y, double c3x, double c3y) {
            this.classId = classId; this.txnc = txnc; this.tync = tync; this.ta = ta;
            corner0_X = c0x; corner0_Y = c0y; corner1_X = c1x; corner1_Y = c1y;
            corner2_X = c2x; corner2_Y = c2y; corner3_X = c3x; corner3_Y = c3y;
        }
    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds, latency, tagSpan, avgTagDist, avgTagArea;
        public int    tagCount;
        public RawFiducial[] rawFiducials;
        public boolean isMegaTag2;

        public PoseEstimate() {
            pose = new Pose2d(); rawFiducials = new RawFiducial[]{}; isMegaTag2 = false;
        }

        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                int tagCount, double tagSpan, double avgTagDist,
                double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {
            this.pose = pose; this.timestampSeconds = timestampSeconds; this.latency = latency;
            this.tagCount = tagCount; this.tagSpan = tagSpan; this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea; this.rawFiducials = rawFiducials; this.isMegaTag2 = isMegaTag2;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            PoseEstimate that = (PoseEstimate) obj;
            return Double.compare(that.latency, latency) == 0 && tagCount == that.tagCount
                && Double.compare(that.tagSpan, tagSpan) == 0
                && Double.compare(that.avgTagDist, avgTagDist) == 0
                && Double.compare(that.avgTagArea, avgTagArea) == 0
                && pose.equals(that.pose) && Arrays.equals(rawFiducials, that.rawFiducials);
        }
    }

    public static class IMUData {
        public double robotYaw=0, Roll=0, Pitch=0, Yaw=0;
        public double gyroX=0, gyroY=0, gyroZ=0;
        public double accelX=0, accelY=0, accelZ=0;
        public IMUData() {}
        public IMUData(double[] d) {
            if (d != null && d.length >= 10) {
                robotYaw=d[0]; Roll=d[1]; Pitch=d[2]; Yaw=d[3];
                gyroX=d[4]; gyroY=d[5]; gyroZ=d[6];
                accelX=d[7]; accelY=d[8]; accelZ=d[9];
            }
        }
    }

    private static ObjectMapper mapper;
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if ("".equals(name) || name == null) return "limelight";
        return name;
    }

    public static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) return new Pose3d();
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]),
                           Units.degreesToRadians(inData[4]),
                           Units.degreesToRadians(inData[5])));
    }

    public static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) return new Pose2d();
        return new Pose2d(new Translation2d(inData[0], inData[1]),
                          new Rotation2d(Units.degreesToRadians(inData[5])));
    }

    public static double[] pose3dToArray(Pose3d pose) {
        return new double[]{
            pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getTranslation().getZ(),
            Units.radiansToDegrees(pose.getRotation().getX()),
            Units.radiansToDegrees(pose.getRotation().getY()),
            Units.radiansToDegrees(pose.getRotation().getZ())
        };
    }

    public static double[] pose2dToArray(Pose2d pose) {
        return new double[]{
            pose.getTranslation().getX(), pose.getTranslation().getY(), 0, 0, 0,
            Units.radiansToDegrees(pose.getRotation().getRadians())
        };
    }

    private static double extractArrayEntry(double[] inData, int position) {
        if (inData.length < position + 1) return 0;
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName, boolean isMegaTag2) {
        DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);
        TimestampedDoubleArray tsValue = poseEntry.getAtomic();
        double[] poseArray = tsValue.value;
        long    timestamp  = tsValue.timestamp;

        if (poseArray.length == 0) return new PoseEstimate();

        var pose       = toPose2D(poseArray);
        double latency = extractArrayEntry(poseArray, 6);
        int    tagCount= (int) extractArrayEntry(poseArray, 7);
        double tagSpan = extractArrayEntry(poseArray, 8);
        double tagDist = extractArrayEntry(poseArray, 9);
        double tagArea = extractArrayEntry(poseArray, 10);
        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

        int valsPerFiducial   = 7;
        int expectedTotalVals = 11 + valsPerFiducial * tagCount;
        RawFiducial[] rawFiducials;

        if (poseArray.length != expectedTotalVals) {
            rawFiducials = new RawFiducial[0];
        } else {
            rawFiducials = new RawFiducial[tagCount];
            for (int i = 0; i < tagCount; i++) {
                int base = 11 + (i * valsPerFiducial);
                rawFiducials[i] = new RawFiducial(
                    (int) poseArray[base], poseArray[base+1], poseArray[base+2],
                    poseArray[base+3], poseArray[base+4], poseArray[base+5], poseArray[base+6]);
            }
        }
        return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, isMegaTag2);
    }

    public static RawFiducial[] getRawFiducials(String limelightName) {
        var entry = getLimelightNTTableEntry(limelightName, "rawfiducials");
        var raw   = entry.getDoubleArray(new double[0]);
        int vpe   = 7;
        if (raw.length % vpe != 0) return new RawFiducial[0];
        RawFiducial[] out = new RawFiducial[raw.length / vpe];
        for (int i = 0; i < out.length; i++) {
            int b = i * vpe;
            out[i] = new RawFiducial((int) raw[b], raw[b+1], raw[b+2], raw[b+3], raw[b+4], raw[b+5], raw[b+6]);
        }
        return out;
    }

    public static RawDetection[] getRawDetections(String limelightName) {
        var entry = getLimelightNTTableEntry(limelightName, "rawdetections");
        var raw   = entry.getDoubleArray(new double[0]);
        int vpe   = 12;
        if (raw.length % vpe != 0) return new RawDetection[0];
        RawDetection[] out = new RawDetection[raw.length / vpe];
        for (int i = 0; i < out.length; i++) {
            int b = i * vpe;
            out[i] = new RawDetection((int) raw[b], raw[b+1], raw[b+2], raw[b+3],
                raw[b+4], raw[b+5], raw[b+6], raw[b+7], raw[b+8], raw[b+9], raw[b+10], raw[b+11]);
        }
        return out;
    }

    public static RawTarget[] getRawTargets(String limelightName) {
        var entry = getLimelightNTTableEntry(limelightName, "rawtargets");
        var raw   = entry.getDoubleArray(new double[0]);
        int vpe   = 3;
        if (raw.length % vpe != 0) return new RawTarget[0];
        RawTarget[] out = new RawTarget[raw.length / vpe];
        for (int i = 0; i < out.length; i++) {
            int b = i * vpe;
            out[i] = new RawTarget(raw[b], raw[b+1], raw[b+2]);
        }
        return out;
    }

    public static double[] getCornerCoordinates(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tcornxy");
    }

    public static void printPoseEstimate(PoseEstimate pose) {
        if (pose == null) { System.out.println("No PoseEstimate available."); return; }
        System.out.printf("Timestamp: %.3f  Latency: %.3f ms  Tags: %d  Span: %.2f m  AvgDist: %.2f m  MT2: %b%n",
            pose.timestampSeconds, pose.latency, pose.tagCount, pose.tagSpan, pose.avgTagDist, pose.isMegaTag2);
        if (pose.rawFiducials == null || pose.rawFiducials.length == 0) return;
        for (int i = 0; i < pose.rawFiducials.length; i++) {
            RawFiducial f = pose.rawFiducials[i];
            System.out.printf("  [%d] id=%d tx=%.2f ty=%.2f dist=%.2f ambig=%.2f%n",
                i+1, f.id, f.txnc, f.tync, f.distToCamera, f.ambiguity);
        }
    }

    public static Boolean validPoseEstimate(PoseEstimate pose) {
        return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static void Flush() { NetworkTableInstance.getDefault().flush(); }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
        String key = tableName + "/" + entryName;
        return doubleArrayEntries.computeIfAbsent(key, k ->
            getLimelightNTTable(tableName).getDoubleArrayTopic(entryName).getEntry(new double[0]));
    }

    public static double   getLimelightNTDouble(String t, String e)      { return getLimelightNTTableEntry(t,e).getDouble(0.0); }
    public static void     setLimelightNTDouble(String t, String e, double v) { getLimelightNTTableEntry(t,e).setDouble(v); }
    public static void     setLimelightNTDoubleArray(String t, String e, double[] v) { getLimelightNTTableEntry(t,e).setDoubleArray(v); }
    public static double[] getLimelightNTDoubleArray(String t, String e)  { return getLimelightNTTableEntry(t,e).getDoubleArray(new double[0]); }
    public static String   getLimelightNTString(String t, String e)       { return getLimelightNTTableEntry(t,e).getString(""); }
    public static String[] getLimelightNTStringArray(String t, String e)  { return getLimelightNTTableEntry(t,e).getStringArray(new String[0]); }

    public static boolean getTV(String n)    { return 1.0 == getLimelightNTDouble(n, "tv"); }
    public static double  getTX(String n)    { return getLimelightNTDouble(n, "tx"); }
    public static double  getTY(String n)    { return getLimelightNTDouble(n, "ty"); }
    public static double  getTXNC(String n)  { return getLimelightNTDouble(n, "txnc"); }
    public static double  getTYNC(String n)  { return getLimelightNTDouble(n, "tync"); }
    public static double  getTA(String n)    { return getLimelightNTDouble(n, "ta"); }
    public static double[] getT2DArray(String n) { return getLimelightNTDoubleArray(n, "t2d"); }

    public static int getTargetCount(String n) {
        double[] t = getT2DArray(n); return t.length == 17 ? (int) t[1] : 0;
    }
    public static int getClassifierClassIndex(String n) {
        double[] t = getT2DArray(n); return t.length == 17 ? (int) t[11] : 0;
    }
    public static int getDetectorClassIndex(String n) {
        double[] t = getT2DArray(n); return t.length == 17 ? (int) t[10] : 0;
    }
    public static String getClassifierClass(String n) { return getLimelightNTString(n, "tcclass"); }
    public static String getDetectorClass(String n)   { return getLimelightNTString(n, "tdclass"); }
    public static double getLatency_Pipeline(String n){ return getLimelightNTDouble(n, "tl"); }
    public static double getLatency_Capture(String n) { return getLimelightNTDouble(n, "cl"); }
    public static double getCurrentPipelineIndex(String n) { return getLimelightNTDouble(n, "getpipe"); }
    public static String getCurrentPipelineType(String n)  { return getLimelightNTString(n, "getpipetype"); }
    public static String getJSONDump(String n)         { return getLimelightNTString(n, "json"); }
    public static double getFiducialID(String n)       { return getLimelightNTDouble(n, "tid"); }
    public static double getHeartbeat(String n)        { return getLimelightNTDouble(n, "hb"); }
    public static String getNeuralClassID(String n)    { return getLimelightNTString(n, "tclass"); }
    public static String[] getRawBarcodeData(String n) { return getLimelightNTStringArray(n, "rawbarcodes"); }
    public static double[] getTargetColor(String n)    { return getLimelightNTDoubleArray(n, "tc"); }

    @Deprecated public static double[] getBotpose(String n)        { return getLimelightNTDoubleArray(n, "botpose"); }
    @Deprecated public static double[] getBotpose_wpiRed(String n) { return getLimelightNTDoubleArray(n, "botpose_wpired"); }
    @Deprecated public static double[] getBotpose_wpiBlue(String n){ return getLimelightNTDoubleArray(n, "botpose_wpiblue"); }

    public static double[] getBotPose(String n)        { return getLimelightNTDoubleArray(n, "botpose"); }
    public static double[] getBotPose_wpiRed(String n) { return getLimelightNTDoubleArray(n, "botpose_wpired"); }
    public static double[] getBotPose_wpiBlue(String n){ return getLimelightNTDoubleArray(n, "botpose_wpiblue"); }
    public static double[] getBotPose_TargetSpace(String n)   { return getLimelightNTDoubleArray(n, "botpose_targetspace"); }
    public static double[] getCameraPose_TargetSpace(String n){ return getLimelightNTDoubleArray(n, "camerapose_targetspace"); }
    public static double[] getTargetPose_CameraSpace(String n){ return getLimelightNTDoubleArray(n, "targetpose_cameraspace"); }
    public static double[] getTargetPose_RobotSpace(String n) { return getLimelightNTDoubleArray(n, "targetpose_robotspace"); }

    public static Pose3d getBotPose3d(String n)         { return toPose3D(getLimelightNTDoubleArray(n,"botpose")); }
    public static Pose3d getBotPose3d_wpiRed(String n)  { return toPose3D(getLimelightNTDoubleArray(n,"botpose_wpired")); }
    public static Pose3d getBotPose3d_wpiBlue(String n) { return toPose3D(getLimelightNTDoubleArray(n,"botpose_wpiblue")); }
    public static Pose3d getBotPose3d_TargetSpace(String n)  { return toPose3D(getLimelightNTDoubleArray(n,"botpose_targetspace")); }
    public static Pose3d getCameraPose3d_TargetSpace(String n){ return toPose3D(getLimelightNTDoubleArray(n,"camerapose_targetspace")); }
    public static Pose3d getTargetPose3d_CameraSpace(String n){ return toPose3D(getLimelightNTDoubleArray(n,"targetpose_cameraspace")); }
    public static Pose3d getTargetPose3d_RobotSpace(String n) { return toPose3D(getLimelightNTDoubleArray(n,"targetpose_robotspace")); }
    public static Pose3d getCameraPose3d_RobotSpace(String n) { return toPose3D(getLimelightNTDoubleArray(n,"camerapose_robotspace")); }

    public static Pose2d getBotPose2d_wpiBlue(String n) { return toPose2D(getBotPose_wpiBlue(n)); }
    public static Pose2d getBotPose2d_wpiRed(String n)  { return toPose2D(getBotPose_wpiRed(n)); }
    public static Pose2d getBotPose2d(String n)         { return toPose2D(getBotPose(n)); }

    public static PoseEstimate getBotPoseEstimate_wpiBlue(String n)         { return getBotPoseEstimate(n, "botpose_wpiblue", false); }
    public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String n){ return getBotPoseEstimate(n, "botpose_orb_wpiblue", true); }
    public static PoseEstimate getBotPoseEstimate_wpiRed(String n)          { return getBotPoseEstimate(n, "botpose_wpired", false); }
    public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String n) { return getBotPoseEstimate(n, "botpose_orb_wpired", true); }

    public static IMUData getIMUData(String n) {
        double[] d = getLimelightNTDoubleArray(n, "imu");
        return (d == null || d.length < 10) ? new IMUData() : new IMUData(d);
    }

    public static void setPipelineIndex(String n, int idx)        { setLimelightNTDouble(n, "pipeline", idx); }
    public static void setPriorityTagID(String n, int id)         { setLimelightNTDouble(n, "priorityid", id); }
    public static void setLEDMode_PipelineControl(String n)       { setLimelightNTDouble(n, "ledMode", 0); }
    public static void setLEDMode_ForceOff(String n)              { setLimelightNTDouble(n, "ledMode", 1); }
    public static void setLEDMode_ForceBlink(String n)            { setLimelightNTDouble(n, "ledMode", 2); }
    public static void setLEDMode_ForceOn(String n)               { setLimelightNTDouble(n, "ledMode", 3); }
    public static void setStreamMode_Standard(String n)           { setLimelightNTDouble(n, "stream", 0); }
    public static void setStreamMode_PiPMain(String n)            { setLimelightNTDouble(n, "stream", 1); }
    public static void setStreamMode_PiPSecondary(String n)       { setLimelightNTDouble(n, "stream", 2); }

    public static void setCropWindow(String n, double xMin, double xMax, double yMin, double yMax) {
        setLimelightNTDoubleArray(n, "crop", new double[]{xMin, xMax, yMin, yMax});
    }
    public static void setKeystone(String n, double h, double v) {
        setLimelightNTDoubleArray(n, "keystone_set", new double[]{h, v});
    }
    public static void setFiducial3DOffset(String n, double x, double y, double z) {
        setLimelightNTDoubleArray(n, "fiducial_offset_set", new double[]{x, y, z});
    }

    public static void SetRobotOrientation(String n, double yaw, double yawRate,
            double pitch, double pitchRate, double roll, double rollRate) {
        SetRobotOrientation_INTERNAL(n, yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
    }
    public static void SetRobotOrientation_NoFlush(String n, double yaw, double yawRate,
            double pitch, double pitchRate, double roll, double rollRate) {
        SetRobotOrientation_INTERNAL(n, yaw, yawRate, pitch, pitchRate, roll, rollRate, false);
    }
    private static void SetRobotOrientation_INTERNAL(String n, double yaw, double yawRate,
            double pitch, double pitchRate, double roll, double rollRate, boolean flush) {
        setLimelightNTDoubleArray(n, "robot_orientation_set",
            new double[]{yaw, yawRate, pitch, pitchRate, roll, rollRate});
        if (flush) Flush();
    }

    public static void SetIMUMode(String n, int mode)       { setLimelightNTDouble(n, "imumode_set", mode); }
    public static void SetIMUAssistAlpha(String n, double a){ setLimelightNTDouble(n, "imuassistalpha_set", a); }
    public static void SetThrottle(String n, int throttle)  { setLimelightNTDouble(n, "throttle_set", throttle); }

    public static void SetFiducialIDFiltersOverride(String n, int[] validIDs) {
        double[] d = new double[validIDs.length];
        for (int i = 0; i < validIDs.length; i++) d[i] = validIDs[i];
        setLimelightNTDoubleArray(n, "fiducial_id_filters_set", d);
    }

    public static void SetFiducialDownscalingOverride(String n, float downscale) {
        int d = 0;
        if (downscale == 1.0f) d=1; else if (downscale == 1.5f) d=2;
        else if (downscale == 2)  d=3; else if (downscale == 3)  d=4;
        else if (downscale == 4)  d=5;
        setLimelightNTDouble(n, "fiducial_downscale_set", d);
    }

    public static void setCameraPose_RobotSpace(String n,
            double forward, double side, double up, double roll, double pitch, double yaw) {
        setLimelightNTDoubleArray(n, "camerapose_robotspace_set",
            new double[]{forward, side, up, roll, pitch, yaw});
    }

    public static void setPythonScriptData(String n, double[] data) { setLimelightNTDoubleArray(n, "llrobot", data); }
    public static double[] getPythonScriptData(String n)            { return getLimelightNTDoubleArray(n, "llpython"); }

    public static void triggerSnapshot(String n) {
        setLimelightNTDouble(n, "snapshot", getLimelightNTDouble(n, "snapshot") + 1);
    }
    public static void setRewindEnabled(String n, boolean enabled) {
        setLimelightNTDouble(n, "rewind_enable_set", enabled ? 1 : 0);
    }
    public static void triggerRewindCapture(String n, double durationSeconds) {
        double[] cur = getLimelightNTDoubleArray(n, "capture_rewind");
        double counter = cur.length > 0 ? cur[0] : 0;
        setLimelightNTDoubleArray(n, "capture_rewind", new double[]{counter + 1, Math.min(durationSeconds, 165)});
    }

    public static LimelightResults getLatestResults(String limelightName) {
        long start = System.nanoTime();
        LimelightResults results = new LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }
        try {
            String json = getJSONDump(limelightName);
            if (json == null || json.isBlank()) {
                results.error = "lljson error: empty json";
            } else {
                results = mapper.readValue(json, LimelightResults.class);
                if (results.imuResults != null) results.imuResults.parseDataArray();
            }
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }
        results.latency_jsonParse = (System.nanoTime() - start) * 0.000001;
        if (profileJSON) System.out.printf("lljson: %.2f\r\n", results.latency_jsonParse);
        return results;
    }

    public static void setupPortForwardingUSB(int usbIndex) {
        String ip      = "172.29." + usbIndex + ".1";
        int    basePort = 5800 + (usbIndex * 10);
        for (int i = 0; i < 10; i++) PortForwarder.add(basePort + i, ip, 5800 + i);
    }
}
