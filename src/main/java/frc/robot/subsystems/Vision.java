// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.Utilities;

// public class Vision extends SubsystemBase {
//         String frontID;

//         PhotonCamera frontLeft, frontRight, backLeft, backRight;
//         Transform3d frontLeftOffset, frontRightOffset, backLeftOffset, backRightOffset;
        
//         AprilTagFieldLayout tagLayout;
//         PhotonPoseEstimator estimator;

//         public enum Camera {
//                 Front,

//                 FrontLeft,
//                 FrontRight,
//                 BackLeft,
//                 BackRight
//         }

//         public Vision(String frontID, String frontLeftID, Transform3d frontLeftOffset, String frontRightID, Transform3d frontRightOffset, String backLeftID, Transform3d backLeftOffset, String backRightID, Transform3d backRightOffset) {
//                 this.frontID = frontID;

//                 frontLeft = new PhotonCamera(frontLeftID);
//                 frontRight = new PhotonCamera(frontRightID);
//                 backLeft = new PhotonCamera(backLeftID);
//                 backRight = new PhotonCamera(backRightID);

//                 this.frontLeftOffset = frontLeftOffset;
//                 this.frontRightOffset = frontRightOffset;
//                 this.backLeftOffset = backLeftOffset;
//                 this.backRightOffset = backRightOffset;

//                 tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
//                 estimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);
//         }

//         PhotonPipelineResult getResult(Camera camera) {
//                 switch (camera) {
//                         case FrontLeft: return frontLeft.getAllUnreadResults().get(0);
//                         case FrontRight: return frontRight.getAllUnreadResults().get(0);
//                         case BackLeft: return backLeft.getAllUnreadResults().get(0);
//                         case BackRight: return backRight.getAllUnreadResults().get(0);

//                         default: return null;
//                 }
//         }

//         Transform3d getOffset(Camera camera) {
//                 switch (camera) {
//                         case FrontLeft: return frontLeftOffset;
//                         case FrontRight: return frontRightOffset;
//                         case BackLeft: return  backLeftOffset;
//                         case BackRight: return backRightOffset;

//                         default: return null;
//                 }
//         }

//         public Pose2d getEstimate(Camera camera) {
//                 Pose2d estimate;
//                 if (camera == Camera.Front) estimate = Utilities.getAlliance() == Alliance.Red ? LimelightHelpers.getRedPoseEstimate(frontID) : LimelightHelpers.getBluePoseEstimate(frontID);

//                 else {
//                         estimator.setRobotToCameraTransform(getOffset(camera));
//                         estimate = estimator.update(getResult(camera)).get().estimatedPose.toPose2d();
//                 }

//                 return estimate;
//         }

//         public Pose2d[] getEstimates() {
//                 Pose2d[] estimates = new Pose2d[] {
//                         getEstimate(Camera.Front),

//                         getEstimate(Camera.FrontLeft),
//                         getEstimate(Camera.FrontRight),
//                         getEstimate(Camera.BackLeft),
//                         getEstimate(Camera.BackRight),
//                 };

//                 return estimates;
//         }
        
//         @Override
//         public void periodic() {}
// }
