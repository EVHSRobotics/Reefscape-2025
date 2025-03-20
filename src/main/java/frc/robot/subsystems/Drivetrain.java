// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Utilities;

import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    static Rotation2d redPerspective = Rotation2d.k180deg, bluePerspective = Rotation2d.kZero;
    boolean appliedPerspective = false;

    SwerveRequest.FieldCentric fieldCentric;
    AutoFactory autoConfigs;

    double percentSpeed;
    // Initial robot pose - used as the relative origin
    private Pose2d initialPose = new Pose2d();
    
    ChassisSpeeds speed;

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
            // Handle exception as needed
            e.printStackTrace();
        }
        
        // Create a supplier for relative robot pose
        Supplier<Pose2d> relativePoseSupplier = this::getRelativePose;
        
        AutoBuilder.configure(
                relativePoseSupplier, // Use relative pose supplier
                this::resetRelativePose, // Method to reset odometry with relative poses
                this::getRobotRelativeSpeeds, 
                (speeds, feedforwards) -> driveRobotRelative(speeds), 
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
        
        autoConfigs = new AutoFactory(
                relativePoseSupplier,
                this::resetRelativePose,
                this::followTrajectory,
                true,
                this
        );
    }

    /**
     * Initialize the initial pose as the current robot pose.
     * Call this method at the start of autonomous to set the relative origin.
     */
    public void initializeRelativePose() {
        initialPose = getRobotPose();
    }
    
    /**
     * Get the robot pose relative to the initial pose.
     * 
     * @return Relative robot pose
     */
    public Pose2d getRelativePose() {
        Pose2d currentPose = getRobotPose();
        
        // Calculate position relative to initial pose
        double relativeX = currentPose.getX() - initialPose.getX();
        double relativeY = currentPose.getY() - initialPose.getY();
        Rotation2d relativeRotation = currentPose.getRotation().minus(initialPose.getRotation());
        
        return new Pose2d(relativeX, relativeY, relativeRotation);
    }
    
    /**
     * Reset the relative pose system to treat the provided pose as relative to the current position.
     * 
     * @param relativePose The pose to set as the new relative position
     */
    public void resetRelativePose(Pose2d relativePose) {
        Pose2d currentPose = getRobotPose();
        
        // Update initial pose so that getRelativePose() will return relativePose
        initialPose = new Pose2d(
            currentPose.getX() - relativePose.getX(),
            currentPose.getY() - relativePose.getY(),
            currentPose.getRotation().minus(relativePose.getRotation())
        );
    }

    /**
     * Get the current absolute pose of the robot.
     * 
     * @return Current robot pose
     */
    public Pose2d getRobotPose() {
        Pose2d estimate = getState().Pose;
        
        return new Pose2d(
                estimate.getX(),
                estimate.getY(),
                getRotation3d().toRotation2d()
        );
    }
    
    /**
     * Get robot-relative speeds from the drivetrain.
     * 
     * @return Robot-relative chassis speeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return speed;
    }

    /**
     * Drive the robot with robot-relative chassis speeds.
     * 
     * @param speeds Robot-relative chassis speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.speed = speeds;
        this.setControl(new SwerveRequest.RobotCentric()
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));
    }
    
    /**
     * Update the robot's speed based on height.
     * 
     * @param height Robot height
     */
    public void updateRobotHeight(double height) {
        percentSpeed = (25 - height) / 25;
    }

    /**
     * Set control of the drivetrain with field-oriented control.
     * 
     * @param speeds Desired chassis speeds
     * @param slowed Whether to apply speed reduction
     */
    public void setControl(ChassisSpeeds speeds, boolean slowed) {
        double velocityX = slowed ? speeds.vxMetersPerSecond * percentSpeed : speeds.vxMetersPerSecond;
        double velocityY = slowed ? speeds.vyMetersPerSecond * percentSpeed : speeds.vyMetersPerSecond;

        setControl(fieldCentric
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Create a command to drive with specified chassis speeds.
     * 
     * @param speeds Desired chassis speeds
     * @param slowed Whether to apply speed reduction
     * @return Command to drive with the specified speeds
     */
    public Command driveSpeeds(ChassisSpeeds speeds, boolean slowed) {
        return run(() -> setControl(speeds, slowed));
    }

    /**
     * Follow a trajectory sample from Choreo.
     * 
     * @param sample Trajectory sample
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d robotPose = getRelativePose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + Constants.Drivetrain.translationPID.calculate(robotPose.getX(), sample.x),
                sample.vy + Constants.Drivetrain.translationPID.calculate(robotPose.getY(), sample.y),
                sample.omega + Constants.Drivetrain.translationPID.calculate(Utilities.getRadians(robotPose), sample.heading)
        );

        setControl(speeds, false);
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
    }
}