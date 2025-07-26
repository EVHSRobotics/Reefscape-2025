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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class QuestNav {

  public Pose2d resetPosition = new Pose2d();

  StructPublisher<Translation2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("translationVector", Translation2d.struct).publish();
  StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault().getStructTopic("QuestNav", Pose2d.struct).publish();



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

  
  public Transform2d questToBot = new Transform2d(0.05,0.274, Rotation2d.fromDegrees(90)).inverse();

  public Translation2d transformer = new Translation2d();
  // Local heading helper variables
  private float yaw_offset = 90.0f;
 
 


  public Pose2d getRobotPose() {

  

    
    if (getBatteryPercent() > 0) 
    {
        //return new Pose2d(getQuestNavTranslation().plus(translator).plus(transformer), getQuestNavPose().getRotation());
        return getQuestNavPose().transformBy(questToBot);
    }
    return null;
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
  public float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  public Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    //questnav2 -> questnav x, quest0 -> quest y
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  


  public Pose2d getQuestNavPose() {
    Pose2d estimate = new Pose2d(getQuestNavTranslation(), Rotation2d.fromDegrees(getOculusYaw()));
    return estimate;
  }


}