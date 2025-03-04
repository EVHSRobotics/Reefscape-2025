// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import com.ctre.phoenix6.hardware.CANrange;
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

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public Outtake outtake = new Outtake();
  public Elevator elevator = new Elevator();

  public StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish();
  public TalonFX outtakemotor = new TalonFX(41);

  public Timer timer;

  public boolean x = false;

  public enum ButtonMode {
    // Coral
    Set_L2(0),
    Set_L3(1),
    Set_L4(2),

    // Algae
    Set_Reef(3),
    Set_Processor(4),

    // Align
    Align_Left(5),
    Align_Center(6),
    Align_Right(7);

    public int id;

    ButtonMode(int id) {
      this.id = id;
    }
  }
  
  ButtonBoard board;


  private final CANdle m_candle = new CANdle(55, "rio");


  String chosenLevel;
  String chosenAlgae;
  String chosenAlignment;

  CANrange canRange;

  public Robot() {

    canRange = new CANrange(32);

    outtakemotor.setNeutralMode(NeutralModeValue.Brake);
    timer = new Timer();
    m_candle.setLEDs(0, 0, 255);

    // board = new ButtonBoard(1);
  }

  ElevatorMode level;
  OuttakeMode angle;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    SmartDashboard.putNumber("outtake position", outtake.arm.getPosition().getValueAsDouble());

    driver.leftBumper().onTrue(outtake.setPosition(OuttakeMode.Stow));
    driver.rightBumper().onTrue(outtake.setPosition(OuttakeMode.Algae));


    driver.a().onTrue(outtake.runCoralIntake());
    driver.x().onTrue(outtake.scoreCoral());


    if (driver.leftTrigger().getAsBoolean()) CommandScheduler.getInstance().cancelAll();

    SmartDashboard.putBoolean("can range", canRange.getIsDetected().getValue());
    SmartDashboard.updateValues();

    // if(board.getButton(ButtonMode.Set_L2)) { chosenLevel = ButtonMode.Set_L2.toString(); level = ElevatorMode.L2_Coral; angle = OuttakeMode.L2_Coral; }
    // if(board.getButton(ButtonMode.Set_L3)) { chosenLevel = ButtonMode.Set_L3.toString(); level = ElevatorMode.L3_Coral; angle = OuttakeMode.L3_Coral; }
    // if(board.getButton(ButtonMode.Set_L4)) { chosenLevel = ButtonMode.Set_L4.toString(); level = ElevatorMode.L4_Coral; angle = OuttakeMode.L4_Coral; }

    // if(board.getButton(ButtonMode.Align_Left)) chosenAlignment = ButtonMode.Align_Left.toString();
    // if(board.getButton(ButtonMode.Align_Center)) chosenAlignment = ButtonMode.Align_Center.toString();
    // if(board.getButton(ButtonMode.Align_Right)) chosenAlignment = ButtonMode.Align_Right.toString();

    // if(board.getButton(ButtonMode.Set_Processor)) chosenAlgae = ButtonMode.Set_Processor.toString();
    // if(board.getButton(ButtonMode.Set_Reef)) chosenAlgae = ButtonMode.Set_Reef.toString();

    // SmartDashboard.putString("Chosen Level", chosenLevel);
    // SmartDashboard.putString("Chosen Algae", chosenAlgae);
    // SmartDashboard.putString("Chosen Alignment", chosenAlignment);
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


    CommandScheduler.getInstance().cancelAll();
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
