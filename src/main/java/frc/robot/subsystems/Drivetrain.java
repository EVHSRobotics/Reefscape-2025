package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.QuestNav;
import frc.robot.Utilities;

import java.util.function.Supplier;


public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    private final PIDController m_pathXController = new PIDController(10, 0, 0);
    private final PIDController m_pathYController = new PIDController(10, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();



    static Rotation2d redPerspective = Rotation2d.k180deg, bluePerspective = Rotation2d.kZero;
    boolean appliedPerspective = false;

    SwerveRequest.FieldCentric fieldCentric;
        public StructPublisher<Pose2d> publisher2;

    AutoFactory autoConfigs;

    double percentSpeed;

    QuestNav questNav = new QuestNav();

    private Pose2d relativePose = new Pose2d();
    

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
            this::resetRelativePose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getStateSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setControl(
                m_pathApplyRobotSpeeds.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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
        return getState().Speeds;
    }

    public void resetRelativePose(Pose2d basePose) {
         relativePose = basePose;

       questNav.setBasePosition(relativePose);
    }

    public Pose2d getRelativePose(){
        return questNav.returnbasePose();
    }

    public Pose2d getRobotPose() {
        Pose2d estimate = questNav.getPose();

        return new Pose2d(
                estimate.getX(),
                estimate.getY(),
                getRotation3d().toRotation2d()
        );
    }

    public Command driveSpeeds(ChassisSpeeds speeds) {
        return driveSpeeds(speeds, false);
}

  public PathPlannerAuto getAutoPath(String autoName) {
        return new PathPlannerAuto(autoName);
    }


    public void bindCommandsAuto(String name, Command command){
        autoConfigs.bind(name, command);
    }

    public Command driveSpeeds(ChassisSpeeds speeds, boolean slowed) {
        return run(() -> setControl(speeds, slowed));
    }

    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return  new AutoFactory(
                ()->getRobotPose(),
                this::resetRelativePose,
                this::followPath,
                true,
                this
        );
    
    }

    public void resetQuestNav(){
        questNav.setBasePosition(new Pose2d());
    }
   
    public Command setZeroSpeed() {
            
                        return driveSpeeds(new ChassisSpeeds());
        }

    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getRobotPose();

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
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

  

public void calculateQuestNavSpeeds(){
    double currentTime = System.currentTimeMillis() / 1000.0;
    
    Pose2d pose = questNav.getPose();
    double dt = currentTime - lastUpdateTime;
    
    // Calculate velocities
    double vx = (pose.getX() - poseforSpeedCalc.getX()) / dt;
    double vy = (pose.getY() - poseforSpeedCalc.getY()) / dt;
    double omega = (pose.getRotation().getRadians() - poseforSpeedCalc.getRotation().getRadians()) / dt;
    
    // Update cache
    questNavSpeed = new ChassisSpeeds(vx, vy, omega);
    
    // Update stored values

    poseforSpeedCalc = getRobotPose();

    lastUpdateTime = currentTime; 
}

public ChassisSpeeds getQuestNavSpeeds(){
    

    return questNavSpeed;
    
}


    @Override
    public void periodic() {

        calculateQuestNavSpeeds();

        if (!appliedPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? redPerspective : bluePerspective
                );

                appliedPerspective = true;
            });
        }

        publisher2.set(getRobotPose());

    }
}