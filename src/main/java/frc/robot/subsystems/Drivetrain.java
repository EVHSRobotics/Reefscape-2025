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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.QuestNav;
import frc.robot.Utilities;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
        SwerveRequest.FieldCentric fieldCentric;
        AutoFactory autoConfigs;

        boolean appliedPerspective = false;
        double antiTipping = 1;

        private final AutoFactory autoFactory;
        
        // Initialize publishers only once
        private final StructArrayPublisher<Pose2d> questNavPublisher;
        private final StructPublisher<Pose2d> encoderPublisher;

        private final QuestNav questNav = new QuestNav();
        private Pose2d robotOdometry;
        private Pose2d basePose;

        // Add these class variables
        private Pose2d lastPositions = new Pose2d(); // x, y, rotation
        private double lastUpdateTime = 0;
        private ChassisSpeeds cachedSpeeds = new ChassisSpeeds(0, 0, 0);
        
        // Add a counter to reduce publishing frequency
        private int displayCounter = 0;
        private static final int DISPLAY_INTERVAL = 5; // Only publish every 5 cycles

        public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfigs, modules);

                // Create publishers once
                questNavPublisher = NetworkTableInstance.getDefault()
                                .getStructArrayTopic("questNav", Pose2d.struct).publish();
                encoderPublisher = NetworkTableInstance.getDefault()
                                .getStructTopic("encoderPosition", Pose2d.struct)
                                .publish();

                robotOdometry = new Pose2d();
                basePose = new Pose2d();
                fieldCentric = new SwerveRequest.FieldCentric()
                                .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
                                .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband)
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                                autoFactory = new AutoFactory(
                                        this::getRobotPose, // A function that returns the current robot pose
                                        this::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
                                        this::followTrajectory, // The drive subsystem trajectory follower 
                                        true, // If alliance flipping should be enabled 
                                        this // The drive subsystem
                                    );

                
        }

        public Pose2d getRobotPose() {
                return questNav.getPose();
        }

        public void resetOdometry(Pose2d pose2d) {
                basePose = pose2d;
                questNav.setBasePosition(basePose);
        }

        public void displayValues() {
                // Only update values at a reduced frequency to avoid subscription limit
                if (displayCounter % DISPLAY_INTERVAL == 0) {
                        questNavPublisher.set(new Pose2d[] { basePose, getRobotPose() });
                        encoderPublisher.set(getState().Pose);
                }
                displayCounter++;
        }




        public void driveRobotRelative(ChassisSpeeds speeds) {
                // Limit dashboard updates to reduce NetworkTables traffic
                if (displayCounter % DISPLAY_INTERVAL == 0) {
                        SmartDashboard.putBoolean("auton running", speeds.vxMetersPerSecond > 0);
                        SmartDashboard.updateValues();
                }

                this.setControl(new SwerveRequest.RobotCentric()
                                .withVelocityX(speeds.vxMetersPerSecond)
                                .withVelocityY(speeds.vyMetersPerSecond)
                                .withRotationalRate(speeds.omegaRadiansPerSecond));
        }

        public void setControl(ChassisSpeeds speeds, boolean slowed) {
                double translationSlow = slowed ? 0.15 : antiTipping;
                double headingSlow = slowed ? 0.5 : antiTipping;

                setControl(fieldCentric
                                .withVelocityX(speeds.vxMetersPerSecond * translationSlow)
                                .withVelocityY(speeds.vyMetersPerSecond * translationSlow)
                                .withRotationalRate(speeds.omegaRadiansPerSecond * headingSlow));
        }

        public AutoFactory getAutoFactory() {
                return autoFactory;
            }

        public Command driveSpeeds(ChassisSpeeds speeds, boolean slowed) {
                return run(() -> setControl(speeds, slowed))
                                .until(() -> true);
        }

        public Command driveSpeeds(ChassisSpeeds speeds) {
                return driveSpeeds(speeds, false);
        }

        public void followTrajectory(SwerveSample sample) {
                Pose2d robotPose = getRobotPose();

                ChassisSpeeds speeds = new ChassisSpeeds(
                                sample.vx + Constants.Drivetrain.translationPID.calculate(robotPose.getX(), sample.x),
                                sample.vy + Constants.Drivetrain.translationPID.calculate(robotPose.getY(), sample.y),
                                sample.omega + Constants.Drivetrain.translationPID
                                                .calculate(Utilities.getRadians(robotPose), sample.heading));

                setControl(speeds, false);
        }

        public void updateRobotHeight(double height) {
                antiTipping = (30 - height) / 30;
        }

        @Override
        public void periodic() {
                SmartDashboard.putBoolean("PoweredON", questNav.getBatteryPercent() > 0);

                
                if (displayCounter % DISPLAY_INTERVAL == 0) {
                        displayCounter = 0;
                        SmartDashboard.updateValues();
                }
                else {
                        displayCounter =+ 1;
                }
                
                if (!appliedPerspective || DriverStation.isDisabled()) {
                        DriverStation.getAlliance().ifPresent(allianceColor -> {
                                setOperatorPerspectiveForward(
                                                allianceColor == Alliance.Red ? Constants.Drivetrain.redPerspective
                                                                : Constants.Drivetrain.bluePerspective);

                                appliedPerspective = true;
                        });
                }
                
                // Update values at a reduced frequency
                displayValues();
        }
}