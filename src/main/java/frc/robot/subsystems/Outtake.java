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

<<<<<<< HEAD
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
import edu.wpi.first.wpilibj.Timer;
>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

<<<<<<< HEAD
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
=======
  public TalonFX intakeMotor = new TalonFX(0);

  public TalonFX pivotMotor = new TalonFX(0);
  public TalonFX outtakeMotor = new TalonFX(0);
  public CANcoder pivotEncoder = new CANcoder(0);

  public CANrange outtakeBanner = new CANrange(0);

  public Timer transferTimer = new Timer();

  public enum OuttakeMode {
    Test_1(0),
    Test_2(0),
    L1_Coral(0),
    L2_Coral(0),
    L3_Coral(0),
    L4_Coral(0),
    Algae(0),
    Source(0),
    Barge(0),
    Hang(0);

    public final double pos;

    private OuttakeMode(double pos) {
        this.pos = pos;
    }
  }

>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45

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

<<<<<<< HEAD
  public Command setPosition(outtakeMode outtakeMode) {
=======
  public Command setPosition(OuttakeMode outtakeMode) {
>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45
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
<<<<<<< HEAD
      public void end(boolean interupted){
=======
      public void execute() {
        pivotMotor.setControl(new MotionMagicExpoVoltage(outtakeMode.pos));
      }
    };
  }

>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45

      }

<<<<<<< HEAD
  };
=======
      @Override
      public void end(boolean interrupted) {
          transferTimer.restart();
          if(!transferTimer.hasElapsed(2)) {
            outtakeMotor.set(-0.01);
          }
          super.end(interrupted);
      }

      @Override
      public void execute() {
        intakeMotor.set(-0.7);
        outtakeMotor.set(-0.3);
      }
    };
  }


  public Command scoreCoral() {
    return new Command() {
      @Override
      public boolean isFinished() {
          return !(outtakeBanner.getDistance().getValueAsDouble() < 10);
      }

      @Override
      public void end(boolean interrupted) {
          super.end(interrupted);
      }

      @Override
      public void execute() {
        outtakeMotor.set(-0.3);
      }
    };
  }
>>>>>>> 51fc17cd4599cd8c85bb1d3bbb7c875d309a6e45

}

public Command[] runIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runIntake'");
}
}
