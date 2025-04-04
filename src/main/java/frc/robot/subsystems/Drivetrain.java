package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.Choreo.TrajectoryLogger;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.QuestNav;
import frc.robot.Utilities;

import java.util.function.Supplier;


public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();


   
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();



    static Rotation2d redPerspective = Rotation2d.k180deg;
    static Rotation2d bluePerspective = Rotation2d.kZero;
    boolean appliedPerspective = false;

    SwerveRequest.FieldCentric fieldCentric;

   



    double percentSpeed;

    QuestNav questNav = new QuestNav();



    private Pose2d basePose2d = new Pose2d();
    //private Transform2d transformer = new Transform2d();
    private Translation2d transformer = new Translation2d();
    

    ChassisSpeeds questNavSpeed = new ChassisSpeeds();
    private Pose2d poseforSpeedCalc = new Pose2d();
    private double lastUpdateTime = 0;


    public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {


        
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfigs, modules);

    



        fieldCentric = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.Drivetrain.maxSpeed * 0.1)
                .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {

            e.printStackTrace();
        }

    AutoBuilder.configure(
            this::getRobotPose, // Robot pose supplier
            this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getStateSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setControl(
                m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond))
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.9, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }
    

    public ChassisSpeeds getStateSpeeds(){
        ChassisSpeeds speed =  getState().Speeds;
        return new ChassisSpeeds(speed.vxMetersPerSecond 
        , speed.vyMetersPerSecond 
        , speed.omegaRadiansPerSecond);
    }

    public void setCurrentPose(Pose2d currentPose){
        resetPose(currentPose);
        transformer = currentPose.getTranslation().minus(questNav.getRobotPose().getTranslation());
    }


    public Pose2d getRobotPose() {
        return new Pose2d(questNav.getRobotPose().getTranslation().plus(transformer), getState().Pose.getRotation());
    }

    public Translation2d geTranslation2d(){
        return transformer;
    }
   

   


    public Command driveSpeeds(ChassisSpeeds speeds) {
        return driveSpeeds(speeds, false);
}

  public PathPlannerAuto getAutoPath(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public Command driveSpeeds(ChassisSpeeds speeds, boolean slowed) {
        return run(() -> setControl(speeds, slowed)).until(() -> true);
    }
 
    public void updateRobotHeight(double height) {
        percentSpeed = (25 - height) / 25;
    }

    public void setControl(ChassisSpeeds speeds, boolean slowed) {
        double velocityX = slowed ? speeds.vxMetersPerSecond * percentSpeed : speeds.vxMetersPerSecond;
        double velocityY = slowed ? speeds.vyMetersPerSecond * percentSpeed : speeds.vyMetersPerSecond;

        setControl(fieldCentric
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

  



    @Override
    public void periodic() {


        if (!appliedPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? redPerspective : bluePerspective
                );

                appliedPerspective = true;
            });
        }

      
        

        SmartDashboard.updateValues();


    }
}