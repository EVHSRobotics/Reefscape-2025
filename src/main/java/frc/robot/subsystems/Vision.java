// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Utilities;

public class Vision extends SubsystemBase {
        String frontID;

        public enum Camera {
                Front
        }

        public Vision(String frontID) {
                this.frontID = frontID;
        }

        String toString(Camera camera) {
                switch (camera) {
                        case Front: return frontID;
                        default: return null;
                }
        }

        public int getTagID(Camera camera) {
                String cameraID = toString(camera);
                return (int)LimelightHelpers.getNTEntry(cameraID, "tid").getInteger(-1);
        }

        public double getOffsetX(Camera camera) {
                String cameraID = toString(camera);
                double offset = Math.tan(Math.toRadians(LimelightHelpers.getNTEntry(cameraID, "tx").getDouble(0))) * getOffsetY(camera);

                return getTagID(camera) != -1 ? offset : 0;
        }

        public double getOffsetY(Camera camera) {
                String cameraID = toString(camera);
                double offset = 0.75 / Math.tan(Math.toRadians(LimelightHelpers.getNTEntry(cameraID, "ty").getDouble(0)));

                return getTagID(camera) != -1 ? offset : 9;
        }

        public Pose2d getEstimate(Camera camera) {
                String cameraID = toString(camera);

                Pose2d redEstimate = LimelightHelpers.getRedPoseEstimate(cameraID);
                Pose2d blueEstimate = LimelightHelpers.getBluePoseEstimate(cameraID);

                return Utilities.getAlliance() == Alliance.Red ? redEstimate : blueEstimate;
        }

        public Pose2d getEstimates() {
                Pose2d estimates = 
                        getEstimate(Camera.Front);

                return estimates;
        }
        
        @Override
        public void periodic() {}
}