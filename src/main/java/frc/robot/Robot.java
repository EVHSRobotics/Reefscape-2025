// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Elevator.ElevatorMode;
import frc.robot.subsystems.Outtake.OuttakeMode;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

  
  public Elevator elevator = new Elevator();
  public Outtake outtake = new Outtake();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity); 

            
  public final CommandXboxController joystick = new CommandXboxController(0);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
  public Pose2d target = new Pose2d();

  public StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();

  public Robot() {  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    publisher2.set(target);

    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
      .withVelocityX(-joystick.getLeftY() * MaxSpeed*0.2) 
      .withVelocityY(-joystick.getLeftX() * MaxSpeed *0.2) 
      .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
    ));

    joystick.y().onTrue(elevator.setPosition(ElevatorMode.Test_1));
    joystick.a().onTrue(elevator.setPosition(ElevatorMode.Test_2));

    joystick.leftBumper().onTrue(outtake.setPosition(OuttakeMode.Test_1));
    joystick.rightBumper().onTrue(outtake.setPosition(OuttakeMode.Test_2));


  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new Command() {
      
    };

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // AutoBuilder.configure(
    //   m_robotContainer.drivetrain::getRobotPose,
    //   m_robotContainer.drivetrain::resetKalman,
    //   m_robotContainer.drivetrain::getSpeeds,
    //   m_robotContainer.drivetrain::driveChassis,
    //   ,
    //   null,
    //   null
    // );
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}