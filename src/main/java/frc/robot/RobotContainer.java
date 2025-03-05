// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorMode;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Outtake.OuttakeMode;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public Outtake outtake;
    public Elevator elevator;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        outtake = new Outtake();
        elevator = new Elevator();
        configureBindings();
    }

    private void configureBindings() {
        driveConfigs();

        driver.y().onTrue(coralScore(OuttakeMode.L3_CORAL, ElevatorMode.L3_Coral));
        driver.a().onTrue(coralScore(OuttakeMode.L2_CORAL, ElevatorMode.L2_Coral));

        driver.leftBumper().onTrue(
                outtake.setPosition(OuttakeMode.STOW).andThen(
                    elevator.setPosition(ElevatorMode.Stow).andThen(
                    outtake.setPosition(OuttakeMode.HARD_STOP).andThen(
                    outtake.runCoralIntake().andThen(
                    outtake.setPosition(OuttakeMode.STOW)
                )))
        ));

        driver.rightBumper().onTrue(
            outtake.scoreCoral()
        );

        driver.b().onTrue(
                outtake.setPosition(OuttakeMode.STOW).andThen(
                    elevator.setPosition(ElevatorMode.Low_Algae).andThen(
                    outtake.setPosition(OuttakeMode.ALGAE).andThen(
                    outtake.runAlgaeIntake()
                ))
        ));

        driver.x().onTrue(
            outtake.setPosition(OuttakeMode.ALGAE).andThen(
                elevator.setPosition(ElevatorMode.Stow).andThen(
                outtake.scoreProcessor()))
        );

        driver.back().onTrue(
            outtake.setPosition(OuttakeMode.STOW).andThen(
                elevator.setPosition(ElevatorMode.Stow)));
    }

    public Command coralScore(OuttakeMode outtakeMode, ElevatorMode elevatorMode) {
        return Commands.sequence(
          elevator.setPosition(elevatorMode),
    
          Commands.waitSeconds(0.1),
    
          outtake.setPosition(outtakeMode),
    
          Commands.waitSeconds(0.1),
    
          outtake.scoreCoral(),
    
          Commands.parallel(
            outtake.setPosition(OuttakeMode.STOW),
            elevator.setPosition(ElevatorMode.Stow)
          )
        );
    }
      
    public void driveConfigs() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        driver.povUp().whileTrue(
            drivetrain.applyRequest(() -> brake)
        );

        driver.povRight().whileTrue(
            drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
            )
        );

        driver.povLeft().onTrue(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        );

        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}