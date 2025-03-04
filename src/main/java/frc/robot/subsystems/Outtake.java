// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {

  public TalonFX arm;
  public TalonFX outtake;
  public CANrange canRange;
  public Timer timer;

  public enum OuttakeMode {
    Stow(0),
    L2_Coral(0),
    L3_Coral(0),
    L4_Coral(0),
    Algae(0.45),
    Barge(0);

    public final double pos;

    private OuttakeMode(double pos) {
        this.pos = pos;
    }
  }


  public Outtake() {
    arm = new TalonFX(Constants.armID);
    outtake = new TalonFX(Constants.outtakeID);
    canRange = new CANrange(Constants.canRangeID);  
    timer = new Timer();

    TalonFXConfiguration outtakeMMConfig = new TalonFXConfiguration();
    Slot0Configs outtakeConfigs = new Slot0Configs();

    outtakeConfigs.kP = 40;


    outtakeMMConfig.Feedback.FeedbackRotorOffset = 0.37;
    outtakeMMConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
    outtakeMMConfig.MotionMagic.MotionMagicAcceleration = 1.5;
    outtakeMMConfig.MotionMagic.MotionMagicJerk = 3;
    outtakeMMConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    outtakeMMConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    outtakeMMConfig.withSlot0(outtakeConfigs);
    arm.getConfigurator().apply(outtakeMMConfig);
  }

  @Override
  public void periodic() {
    if (timer.get() < 0.5) {
      outtake.set(-0.1);
    }

    else if (timer.get() > 0.5) {
      outtake.set(0);
      timer.stop();
    }
  }

  public Command setPosition(OuttakeMode outtakeMode) {
    return new Command() {
      @Override
      public void execute() {
        arm.setControl(new MotionMagicExpoVoltage(outtakeMode.pos));
      }

      @Override
      public boolean isFinished() {
        return true;
      }

      @Override
      public void end(boolean interrupted) {
        super.end(interrupted);
      }


      @Override
      public void initialize() {
        addRequirements(new Outtake());
      }
    };
  }

  public Command runCoralIntake() {
    return new Command() {
      @Override
      public void execute() {
        outtake.set(-0.4);
      }

      @Override
      public boolean isFinished() {
        return canRange.getIsDetected(true).getValue();
      }

      @Override
      public void end(boolean interupted){
        outtake.set(0);
        timer.restart();
      }

    };
  }

  public Command runAlgaeIntake() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return outtake.getMotorVoltage().getValueAsDouble() > 40;
      }

      @Override
      public void end(boolean interupted){
        super.end(interupted);
      }

      @Override
      public void execute() {
        outtake.set(0.4);
      }
    };
  }


  public Command scoreCoral() {
    return new Command() {

      @Override
      public void execute() {
        outtake.set(-1);
      }

      @Override
      public boolean isFinished() {
        return !canRange.getIsDetected().getValue();
      }

      @Override
      public void end(boolean interrupted) {
        outtake.set(0);
      }
    };
  }

}