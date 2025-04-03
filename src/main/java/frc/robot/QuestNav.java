package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class QuestNav {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();
  
  // Quest offset from robot center (to be calibrated)
  private Translation2d questOffset = new Translation2d(0, 0.1651); // Default offset, will be calibrated

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    Pose2d rawQuestPose = getQuestNavPose();
    // Apply the transformer to get robot center pose (compensating for offset)
    return compensateForQuestOffset(rawQuestPose);
  }

  private Pose2d compensateForQuestOffset(Pose2d questPose) {
    Translation2d rotatedOffset = questOffset.rotateBy(questPose.getRotation());
    
    Translation2d robotCenter = questPose.getTranslation().minus(rotatedOffset);
    
    return new Pose2d(robotCenter, questPose.getRotation());
  }


  public void calibrateQuestOffset(Pose2d pose1, Pose2d pose2) {
    double angleDiff = Math.abs(pose1.getRotation().minus(pose2.getRotation()).getDegrees());
    if (angleDiff < 20) {

    
    
    Rotation2d rotationDiff = pose2.getRotation().minus(pose1.getRotation());
    
    Translation2d trans1 = pose1.getTranslation();
    Translation2d trans2 = pose2.getTranslation();
    
    Translation2d relativeMovement = trans2.minus(trans1);
    
    double d = rotationDiff.getRadians();
    
    if (Math.abs(d) < 0.01) {
    
    double offsetMagnitude = relativeMovement.getNorm() / (2 * Math.sin(d/2));
    
    Rotation2d offsetDir = new Rotation2d(relativeMovement.getY(), -relativeMovement.getX());
    offsetDir = offsetDir.plus(new Rotation2d(Math.PI/2)); 
    
    this.questOffset = new Translation2d(
        offsetMagnitude * offsetDir.getCos(),
        offsetMagnitude * offsetDir.getSin());
    
    SmartDashboard.putNumber("Quest offset x ", this.questOffset.getX());
    SmartDashboard.putNumber("Quest offset y ", this.questOffset.getY());
  }}

  }

  public Translation2d getQuestOffset() {
    return questOffset;
  }




  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp in NT Server Time.
  public double timestamp() {
    return questTimestamp.getAtomic().serverTime;
  }

  // Zero the relativerobot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  public void resetFullPosition(){
    zeroHeading();
    zeroPosition();
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  public Pose2d getQuestNavPose() {
    // Get raw Quest position without offset compensation
    var rawQuestPosition = getQuestNavTranslation();
    // Note: This is just the raw reading, without compensating for the offset
    return new Pose2d(rawQuestPosition, Rotation2d.fromDegrees(getOculusYaw()));
  }
}