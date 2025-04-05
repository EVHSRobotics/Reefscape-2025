// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;

public class Constants {
        public static int controllerID = 0, boardID = 1;
        public static int lightsID = 55;

        public static String canivoreID = "*";

        public static double deadband = 0.1;

        public static Pose2d[] leftTargets = new Pose2d[] {
                new Pose2d(3.13, 4.19, new Rotation2d(0)),
                new Pose2d(3.95, 5.28, new Rotation2d(-60)),
                new Pose2d(5.31, 5.12, new Rotation2d(-120)),
                new Pose2d(5.85, 3.86, new Rotation2d(180)),
                new Pose2d(5.02, 2.76, new Rotation2d(120)),
                new Pose2d(3.67, 2.93, new Rotation2d(60))
        };

        public static Pose2d[] rightTargets = new Pose2d[] {
                new Pose2d(3.13, 3.86, new Rotation2d(0)),
                new Pose2d(3.67, 5.12, new Rotation2d(-60)),
                new Pose2d(5.02, 5.28, new Rotation2d(-120)),
                new Pose2d(5.85, 4.19, new Rotation2d(180)),
                new Pose2d(5.31, 2.93, new Rotation2d(120)),
                new Pose2d(3.95, 2.76, new Rotation2d(60))
        };

        public class Drivetrain {
                static int frontLeftDriveID = 5, frontLeftSteerID = 6, frontLeftEncoderID = 52;
                static int frontRightDriveID = 3, frontRightSteerID = 4, frontRightEncoderID = 51;
                static int backLeftDriveID = 7, backLeftSteerID = 8, backLeftEncoderID = 54;
                static int backRightDriveID = 1, backRightSteerID = 2, backRightEncoderID = 53;

                static int gyroID = 13;

                public static Rotation2d redPerspective = Rotation2d.kZero, bluePerspective = Rotation2d.k180deg;
                public static PIDController translationPID = new PIDController(5, 0, 0); 
                public static PIDController headingPID = new PIDController(5, 0, 0); 

                static TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
                static TalonFXConfiguration steerConfigs = new TalonFXConfiguration().withCurrentLimits(
                        new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true)
                );

                static CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
                static Pigeon2Configuration pigeonConfigs = null;

                static Slot0Configs driveGains = new Slot0Configs()
                        .withKP(0.1).withKI(0).withKD(0)
                        .withKS(0).withKV(0.124);

                static Slot0Configs steerGains = new Slot0Configs()
                        .withKP(100).withKI(0).withKD(0.5)
                        .withKS(0.1).withKV(2.66).withKA(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

                static ClosedLoopOutputType driveOutputType = ClosedLoopOutputType.Voltage;
                static ClosedLoopOutputType steerOutputType = ClosedLoopOutputType.Voltage;

                static DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
                static SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

                static SteerFeedbackType feedbackType = SteerFeedbackType.FusedCANcoder;

                static Current slipCurrent = Amps.of(80);

                public static double maxSpeed = 4.73;
                public static double maxAcceleration = 9.81;
                public static double maxAngularSpeed = 8.24;
                public static double maxAngularAcceleration = 38.8;

                static double coupleRatio = 3.5714285714285716;
                static double driveRatio = 6.746031746031747;
                static double steerRatio = 21.428571428571427;

                static Distance wheelRadius = Inches.of(2);

                static Voltage driveFriction = Volts.of(0.2);
                static Voltage steerFriction = Volts.of(0.2);

                static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(driveRatio)
                        .withSteerMotorGearRatio(steerRatio)
                        .withCouplingGearRatio(coupleRatio)
                        .withWheelRadius(wheelRadius)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerOutputType)
                        .withDriveMotorClosedLoopOutput(driveOutputType)
                        .withSlipCurrent(slipCurrent)
                        .withSpeedAt12Volts(maxSpeed)
                        .withDriveMotorType(driveMotorType)
                        .withSteerMotorType(steerMotorType)
                        .withFeedbackSource(feedbackType)
                        .withDriveMotorInitialConfigs(driveConfigs)
                        .withSteerMotorInitialConfigs(steerConfigs)
                        .withEncoderInitialConfigs(encoderConfigs)
                        .withSteerFrictionVoltage(steerFriction)
                        .withDriveFrictionVoltage(driveFriction);

                static Angle frontLeftOffset = Rotations.of(0.036865234375);
                static Distance frontLeftX = Inches.of(12.625), frontLeftY = Inches.of(11.3125);

                static Angle frontRightOffset = Rotations.of(-0.439208984375);
                static Distance frontRightX = Inches.of(12.625), frontRightY = Inches.of(-11.3125);

                static Angle backLeftOffset = Rotations.of(-0.34912109375);
                static Distance backLeftX = Inches.of(-12.625), backLeftY = Inches.of(11.3125);

                static Angle backRightOffset = Rotations.of(0.36181640625);
                static Distance backRightX = Inches.of(-12.625), backRightY = Inches.of(-11.3125);

                public static SwerveDrivetrainConstants drivetrainConfigs = new SwerveDrivetrainConstants()
                        .withCANBusName(canivoreID)
                        .withPigeon2Id(gyroID)
                        .withPigeon2Configs(pigeonConfigs);

                public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftConfigs = ConstantCreator.createModuleConstants(
                        frontLeftSteerID, frontLeftDriveID, frontLeftEncoderID, frontLeftOffset,
                        frontLeftX, frontLeftY, false, true, false
                );

                public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightConfigs = ConstantCreator.createModuleConstants(
                        frontRightSteerID, frontRightDriveID, frontRightEncoderID, frontRightOffset,
                        frontRightX, frontRightY, true, true, false
                );

                public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftConfigs = ConstantCreator.createModuleConstants(
                        backLeftSteerID, backLeftDriveID, backLeftEncoderID, backLeftOffset,
                        backLeftX, backLeftY, false, true, false
                );

                public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightConfigs = ConstantCreator.createModuleConstants(
                        backRightSteerID, backRightDriveID, backRightEncoderID, backRightOffset,
                        backRightX, backRightY, true, true, false
                );
        }

        public class Arm {
                public static int pivotID = 30, rollersID = 41;
                public static int coralRangeID = 32, algaeRangeID = 33;
        }

        public class Elevator {
                public static int leftID = 20, rightID = 21;
        }

        public class Climber {
                public static int winchID = 60;
        }

        public class Vision {
                public static String frontID = "limelight-center";
        }

        public class Quest {
                public static Translation2d positionOffset = new Translation2d();
                public static double rotationOffset = 0;
        }
}