// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

  DigitalInput banner = new DigitalInput(8);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity); 

            
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public Outtake outtake = new Outtake();

  public StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();
  public TalonFX outtakemotor = new TalonFX(41);

  public Timer timer;

  public boolean x = false;

  
  private final CANdle m_candle = new CANdle(55, "rio");

  public Robot() {
    outtakemotor.setNeutralMode(NeutralModeValue.Brake);
    timer = new Timer();
    m_candle.setLEDs(0, 0, 255);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // SmartDashboard.putNumber("outtake motor voltage", outtake.outtake.getMotorVoltage().getValueAsDouble());

    // driver.leftBumper().onTrue(outtake.setPosition(OuttakeMode.Stow));
    // driver.rightBsumper().onTrue(outtake.setPosition(OuttakeMode.Algae));
    SmartDashboard.putBoolean("banner", banner.get());


    
    if(driver.a().getAsBoolean() && !banner.get()) {
      outtakemotor.set(-0.3);
      m_candle.setLEDs(255, 0, 0);
      x = true;
    }

    

    if(banner.get() && x) {
      timer.restart();
      if(timer.get() < 1.5) {
        outtakemotor.set(-0.01);
      } else {
        outtakemotor.set(0);    
        m_candle.setLEDs(0, 255, 0);
        x = false;
        timer.stop();
      }
    }

    if(driver.b().getAsBoolean()) {
      outtakemotor.set(-0.7);
    } else {
      outtakemotor.set(0);
    }
    
    // SmartDashboard.putBoolean("banner", banner.get());
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
