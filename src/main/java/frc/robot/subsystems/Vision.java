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

        public Pose2d getEstimate(Camera camera) {
                String cameraID = toString(camera);

                Pose2d redEstimate = LimelightHelpers.getBotPose2d_wpiRed(cameraID);
                Pose2d blueEstimate = LimelightHelpers.getBotPose2d_wpiBlue(cameraID);

                return Utilities.getAlliance() == Alliance.Red ? redEstimate : blueEstimate;
        }

        public Pose2d[] getEstimates() {
                Pose2d[] estimates = new Pose2d[] {
                        getEstimate(Camera.Front),
                };

                return estimates;
        }
        
        @Override
        public void periodic() {}
}
