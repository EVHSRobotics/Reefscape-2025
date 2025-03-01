// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public PhotonCamera frontRightCam, backRightCam, backLeftCam, frontLeftCam;

  public AprilTagFieldLayout ATlayout;
  public PhotonPoseEstimator frontRightPoseEstimator, backRightPoseEstimator, frontLeftPoseEstimator, backLeftPoseEstimator;
  public Transform3d frontRightCamPos, backRightCamPos, backLeftCamPos, frontLeftCamPos;

  public enum Camera {
    FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
  }

  public Vision() {    
    ATlayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    frontRightCam = new PhotonCamera("frontRightCam");
    frontRightCamPos = new Transform3d(new Translation3d(0.3175, -0.2794, 0.136525), new Rotation3d(0, 0, 0));
    frontRightPoseEstimator = new PhotonPoseEstimator(ATlayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightCamPos);

    backRightCam = new PhotonCamera("backRightCam");
    backRightCamPos = new Transform3d(new Translation3d(-0.3175, -0.2794, 0.136525), new Rotation3d(0, 0, Math.PI * 0.5));
    backRightPoseEstimator = new PhotonPoseEstimator(ATlayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightCamPos);

    backLeftCam = new PhotonCamera("frontLeftCam");
    backLeftCamPos = new Transform3d(new Translation3d(-0.3175, 0.2794, 0.136525), new Rotation3d(0, 0, Math.PI));
    backLeftPoseEstimator = new PhotonPoseEstimator(ATlayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftCamPos);

    frontLeftCam = new PhotonCamera("backLeftCam");
    frontLeftCamPos = new Transform3d(new Translation3d(0.3175, 0.2794, 0.136525), new Rotation3d(0, 0, Math.PI * 1.5));
    frontLeftPoseEstimator = new PhotonPoseEstimator(ATlayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftCamPos);
  }

  public Pose2d getCameraPose(Camera requestedCamera, Pose2d currentPose) {
    PhotonCamera camera;
    
    switch(requestedCamera) {
      case FRONT_LEFT:
      camera = frontLeftCam;
        break;

      case FRONT_RIGHT:
      camera = frontRightCam;
        break;
        
      case BACK_LEFT:
      camera = backLeftCam;
        break;

      default:
      camera = backRightCam;
        break;
    }

    Transform3d currentPose3d = camera.getLatestResult().getMultiTagResult().get().estimatedPose.best;
    return new Pose2d(currentPose3d.getX(), currentPose3d.getY(),new Rotation2d(currentPose3d.getRotation().getX(), currentPose3d.getRotation().getY()));
  }

  @Override
  public void periodic() {}
}
