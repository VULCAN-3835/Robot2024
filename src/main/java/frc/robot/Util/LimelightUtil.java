// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightUtil {
    // Limelight
    private NetworkTable limelight;
    public LimelightUtil(String limelightName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    }
    // Returns the x offset of the center of camera from target
    public double getX() {
        return limelight.getEntry("tx").getDouble(0);
    }
    // Returns the y offset of the center of camera from target
    public double getY() {
        return limelight.getEntry("ty").getDouble(0);
    }
    // Returns how much of screen is taken
    public double getA() {
        return limelight.getEntry("ta").getDouble(0);
    }
    // Returns true if camera has target
    public boolean cameraHasTarget() {
        return this.limelight.getEntry("tv").getDouble(0) != 0;
    }
    // Changes the camera to april tag detection pipeline
    public void setAprilMode() {
        limelight.getEntry("pipeline").setNumber(0);
    }
    // Changes the camera to reflection detection pipeline
    public void setReflectorMode() {
        limelight.getEntry("pipeline").setNumber(1);
    }

    // Returns the Pose2d relative to blue corner
    public Pose2d getPoseFromCamera() {
        double[] botpose = this.limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5];
        return new Pose2d(x,y,Rotation2d.fromDegrees(yaw));
    }

    // Returns the total latency of limelight camera
    public double getCameraTimeStampSec() {
        double timestamp = this.limelight.getEntry("cl").getDouble(0)+this.limelight.getEntry("tl").getDouble(0);
        return timestamp/1000;
    }

    // Returns the distance from current april tag in meters
    public double distanceFromTargetMeters() {
        double[] botpose_apriltag = this.limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        double z = botpose_apriltag[2];
        double x = botpose_apriltag[0];

        return Math.sqrt(Math.pow(z,2)+Math.pow(x,2));
    }

    // Returns if limelight has valid target
    public boolean hasValidTarget() {
        return cameraHasTarget() && distanceFromTargetMeters() < 3.4;
    }

    public double getAprilTagID() {
        return this.limelight.getEntry("tid").getDouble(0);
    }
}