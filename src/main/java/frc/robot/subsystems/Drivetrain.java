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

    static Rotation2d redPerspective = Rotation2d.k180deg, bluePerspective = Rotation2d.kZero;
    boolean appliedPerspective = false;

    SwerveRequest.FieldCentric fieldCentric;
        public StructPublisher<Pose2d> publisher2;

    AutoFactory autoConfigs;

    double percentSpeed;

    QuestNav questNav = new QuestNav();

    private Pose2d relativePose = new Pose2d();

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

        publisher2 = NetworkTableInstance.getDefault()
      .getStructTopic("Questnav position", Pose2d.struct)
      .publish();  

        Supplier<Pose2d> relativePoseSupplier = this::getRobotPose;

        autoConfigs = new AutoFactory(
                relativePoseSupplier,
                this::resetRelativePose,
                this::followPath,
                true,
                this
        );
    }

    public void resetRelativePose(Pose2d relativePose) {
         this.relativePose = relativePose;

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

    public Command startTrajectory(String trajectory) {
        return autoConfigs.resetOdometry(trajectory);
}

public Command followTrajectory(String trajectory) {
        return Commands.sequence(
                autoConfigs.trajectoryCmd(trajectory),
                driveSpeeds(new ChassisSpeeds())
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

        publisher2.set(getRobotPose());

    }
}