package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

<<<<<<< HEAD
  public TalonFX elevator1;
  public TalonFX elevator2;
  public int elevator1Id;
  public int elevator2Id;

  public enum elevatorMode {
    L1(10), 
    L2(30), 
    L3(40),
    L4(50),
    CORAL_INTAKE(0);
=======
  public TalonFX left = new TalonFX(21);
  public TalonFX right = new TalonFX(20);


  public enum ElevatorMode {
    Test_1(0),
    Test_2(0),
    L1_Coral(0),
    L2_Coral(0),
    L3_Coral(0),
    L4_Coral(0),
    L2_Algae(0),
    L3_Algae(0),
    Source(0),
    Processor(0),
    Barge(0),
    Hang(0);

    public final double pos;

    private ElevatorMode(double pos) {
        this.pos = pos;
    }
  }
>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45

    private double position;


    elevatorMode(double position) {
      this.position = position;
    }
  }

  public Elevator() {
    elevator1 = new TalonFX(elevator1Id);
    elevator2 = new TalonFX(elevator2Id);

    TalonFXConfiguration elevatorMMConfig = new TalonFXConfiguration();
    Slot0Configs elevatorConfigs = new Slot0Configs();

    elevatorConfigs.kP = 5;
    elevatorConfigs.kI = 0;
    elevatorConfigs.kD = 0;

    elevatorConfigs.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfigs.kG = 1;

    elevatorMMConfig.CurrentLimits.SupplyCurrentLimit = 80;
    elevatorMMConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorMMConfig.CurrentLimits.StatorCurrentLimit = 80;
    elevatorMMConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorMMConfig.Feedback.FeedbackRemoteSensorID = elevator1.getDeviceID();
    elevatorMMConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elevatorMMConfig.MotionMagic.MotionMagicAcceleration = 1;
    elevatorMMConfig.MotionMagic.MotionMagicCruiseVelocity = 0.7;
    elevatorMMConfig.MotionMagic.MotionMagicJerk = 0.1;
    elevatorMMConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMMConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMMConfig.withSlot0(elevatorConfigs);

    elevator1.getConfigurator().apply(elevatorMMConfig);
    elevator2.getConfigurator().apply(elevatorMMConfig);
    
    elevator2.setControl(new Follower(elevator1Id, false));
  }

  @Override
  public void periodic() {}
<<<<<<< HEAD
  public Command setPosition(elevatorMode elevatorMode) {
=======

  public Command setPosition(ElevatorMode elevatorMode) {
>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45
    return new Command() {
      @Override
      public void execute(){
        elevator1.setControl(new MotionMagicExpoVoltage(elevatorMode.position));
      }

      @Override
      public boolean isFinished(){
        return true;
      }

      @Override
<<<<<<< HEAD
      public void end(boolean interupted){
=======
      public void execute() {
        left.setControl(new MotionMagicExpoVoltage(elevatorMode.pos));
      }
    };
  }



>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45

      }

  };

}
}
