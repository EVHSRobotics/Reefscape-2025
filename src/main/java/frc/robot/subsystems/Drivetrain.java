// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.QuestNav;
import frc.robot.Utilities;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
        SwerveRequest.FieldCentric fieldCentric;
        SwerveRequest.RobotCentric robotCentric;

        RobotConfig config;

        boolean appliedPerspective = false;
        double antiTipping = 1;
        public QuestNav questNav;

        public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfigs, modules);

                fieldCentric = new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
                        .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);

                robotCentric = new SwerveRequest.RobotCentric()
                        .withDeadband(Constants.Drivetrain.maxSpeed * Constants.deadband)
                        .withRotationalDeadband(Constants.Drivetrain.maxAngularSpeed * Constants.deadband);

                try {
                        config = RobotConfig.fromGUISettings();
                }
                catch (Exception err) {
                        err.printStackTrace();
                }

                questNav = new QuestNav();

                AutoBuilder.configure(
                        this::getRobotPose,
                        this::setCurrentPose,
                        () -> getState().Speeds,
                        (speeds, feedforwards) -> setRobotControl(speeds),
                        new PPHolonomicDriveController(
                                new PIDConstants(Constants.Drivetrain.translationP, Constants.Drivetrain.translationI, Constants.Drivetrain.translationD),
                                new PIDConstants(Constants.Drivetrain.headingP, Constants.Drivetrain.headingI, Constants.Drivetrain.headingD)
                        ),
                        config,
                        () -> Utilities.getAlliance() == Alliance.Red
                );
        }

        public void setCurrentPose(Pose2d currentPose){
                resetPose(currentPose);
                questNav.setPose(currentPose); 
        }
        public Pose2d getRobotPose() {


                Pose2d estimate = getState().Pose;
                
                return new Pose2d(
                        estimate.getX(),
                        estimate.getY(),
                        getPigeon2().getRotation2d()
                );
        }

        public void updateRobotHeight(double height) {
                antiTipping = (30 - height) / 30;
        }

        public void setFieldControl(ChassisSpeeds speeds, boolean slowed) {
                double translationSlow = slowed ? 0.15 : antiTipping;
                double headingSlow = slowed ? 0.5 : antiTipping;

                setControl(fieldCentric
                        .withVelocityX(speeds.vxMetersPerSecond * translationSlow)
                        .withVelocityY(speeds.vyMetersPerSecond * translationSlow)
                        .withRotationalRate(speeds.omegaRadiansPerSecond * headingSlow)
                );
        }

        public void setRobotControl(ChassisSpeeds speeds) {
                setControl(robotCentric
                        .withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond)
                );
        }

        public Command driveSpeeds(ChassisSpeeds speeds, boolean slowed) {
                return run(() -> setFieldControl(speeds, slowed))
                .until(() -> true);
        }

        public Command driveSpeeds(ChassisSpeeds speeds) {
                return driveSpeeds(speeds, false);
        }

        @Override
        public void periodic() {
                if (!appliedPerspective || DriverStation.isDisabled()) {
                        DriverStation.getAlliance().ifPresent(allianceColor -> {
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red ? Constants.Drivetrain.redPerspective : Constants.Drivetrain.bluePerspective
                                );

                                appliedPerspective = true;
                        });
                }
        }
}
