// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

public class Constants {
        public static int controllerID = 0, boardID = 1;
        public static double deadband = 0.15;

        public static String canivoreID = "*";
        public static int lightsID = 55;

        public static Pose2d[] centerTargets = new Pose2d[] {
                new Pose2d(3.15, 4.03, Rotation2d.fromDegrees(0)),
                new Pose2d(3.81, 5.18, Rotation2d.fromDegrees(-60)),
                new Pose2d(5.16, 5.18, Rotation2d.fromDegrees(-120)),
                new Pose2d(5.83, 4.03, Rotation2d.fromDegrees(180)),
                new Pose2d(5.16, 2.87, Rotation2d.fromDegrees(120)),
                new Pose2d(3.81, 2.87, Rotation2d.fromDegrees(60)),

                new Pose2d(11.73, 4.03, Rotation2d.fromDegrees(0)),
                new Pose2d(12.4, 5.18, Rotation2d.fromDegrees(-60)),
                new Pose2d(13.73, 5.18, Rotation2d.fromDegrees(-120)),
                new Pose2d(14.4, 4.03, Rotation2d.fromDegrees(180)),
                new Pose2d(13.73, 2.87, Rotation2d.fromDegrees(120)),
                new Pose2d(12.4, 2.87, Rotation2d.fromDegrees(60))
        };

        public static Pose2d[] leftTargets = new Pose2d[] {
                new Pose2d(3.15, 4.19, Rotation2d.fromDegrees(0)),
                new Pose2d(3.96, 5.26, Rotation2d.fromDegrees(-60)),
                new Pose2d(5.3, 5.1, Rotation2d.fromDegrees(-120)),
                new Pose2d(5.82, 3.86, Rotation2d.fromDegrees(180)),
                new Pose2d(5.01, 2.78, Rotation2d.fromDegrees(120)),
                new Pose2d(3.68, 2.95, Rotation2d.fromDegrees(60)),
                
                new Pose2d(11.73, 4.19, Rotation2d.fromDegrees(0)),
                new Pose2d(12.54, 5.26, Rotation2d.fromDegrees(-60)),
                new Pose2d(13.88, 5.1, Rotation2d.fromDegrees(-120)),
                new Pose2d(14.4, 3.86, Rotation2d.fromDegrees(180)),
                new Pose2d(13.59, 2.78, Rotation2d.fromDegrees(120)),
                new Pose2d(12.25, 2.95, Rotation2d.fromDegrees(60))
        };

        public static Pose2d[] rightTargets = new Pose2d[] {
                new Pose2d(3.15, 3.86, Rotation2d.fromDegrees(0)),
                new Pose2d(3.68, 5.1, Rotation2d.fromDegrees(-60)),
                new Pose2d(5.01, 5.26, Rotation2d.fromDegrees(-120)),
                new Pose2d(5.82, 4.19, Rotation2d.fromDegrees(180)),
                new Pose2d(5.3, 2.95, Rotation2d.fromDegrees(120)),
                new Pose2d(3.96, 2.78, Rotation2d.fromDegrees(60)),
                
                new Pose2d(11.73, 3.86, Rotation2d.fromDegrees(0)),
                new Pose2d(12.25, 5.1, Rotation2d.fromDegrees(-60)),
                new Pose2d(13.59, 5.26, Rotation2d.fromDegrees(-120)),
                new Pose2d(14.4, 4.19, Rotation2d.fromDegrees(180)),
                new Pose2d(13.88, 2.95, Rotation2d.fromDegrees(120)),
                new Pose2d(12.54, 2.78, Rotation2d.fromDegrees(60))
        };

        public class Drivetrain {
                static int gyroID = 13;

                static int frontLeftDriveID = 5, frontLeftSteerID = 6, frontLeftEncoderID = 52;
                static int frontRightDriveID = 3, frontRightSteerID = 4, frontRightEncoderID = 51;
                static int backLeftDriveID = 7, backLeftSteerID = 8, backLeftEncoderID = 54;
                static int backRightDriveID = 1, backRightSteerID = 2, backRightEncoderID = 53;

                public static Rotation2d redPerspective = Rotation2d.kZero, bluePerspective = Rotation2d.k180deg;
                
                public static double translationP = 10, translationI = 0, translationD = 0;
                public static double headingP = 12, headingI = 0, headingD = 0;

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
}
