package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class Outtake extends SubsystemBase {

  public TalonFX outtakeMotor;
  public TalonFX outtakeRollers;
  public int outtakeMotorID;
  public int OuttakeRollersID;

  public enum outtakeMode {
    L1(10), 
    L2(30), 
    L3(40),
    L4(50),
    ALGAE_INTAKE(1),
    CORAL_INTAKE(0),
    Stow(0);

    public double position;

    outtakeMode(double position) {
      this.position = position;
    }
  }

  public Outtake() {
    outtakeMotor = new TalonFX(outtakeMotorID);
    outtakeRollers = new TalonFX(OuttakeRollersID);

    TalonFXConfiguration outtakeMMConfig = new TalonFXConfiguration();
    Slot0Configs outtakeConfigs = new Slot0Configs();

    outtakeConfigs.kP = 5;
    outtakeConfigs.kI = 0;
    outtakeConfigs.kD = 0;

    outtakeConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    outtakeConfigs.kG = 1;

    outtakeMMConfig.CurrentLimits.SupplyCurrentLimit = 80;
    outtakeMMConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    outtakeMMConfig.CurrentLimits.StatorCurrentLimit = 80;
    outtakeMMConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    outtakeMMConfig.Feedback.FeedbackRemoteSensorID = outtakeMotor.getDeviceID();
    outtakeMMConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    outtakeMMConfig.MotionMagic.MotionMagicAcceleration = 1;
    outtakeMMConfig.MotionMagic.MotionMagicCruiseVelocity = 0.7;
    outtakeMMConfig.MotionMagic.MotionMagicJerk = 0.1;
    outtakeMMConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    outtakeMMConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outtakeMMConfig.withSlot0(outtakeConfigs);

    outtakeMotor.getConfigurator().apply(outtakeMMConfig);
    
  }

  @Override
  public void periodic() {}

  public Command setPosition(outtakeMode outtakeMode) {
    return new Command() {
      @Override
      public void execute(){
        outtakeMotor.setControl(new MotionMagicExpoVoltage(outtakeMode.position));
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

public Command[] runIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runIntake'");
}
}
