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
  public Command setPosition(elevatorMode elevatorMode) {
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
      public void end(boolean interupted){

      }

  };

}
}
