package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

  public TalonFX intakeMotor = new TalonFX(0);

  public TalonFX pivotMotor = new TalonFX(0);
  public TalonFX outtakeMotor = new TalonFX(0);
  public CANcoder pivotEncoder = new CANcoder(0);

  public CANrange outtakeBanner = new CANrange(0);

  public Timer transferTimer = new Timer();


  public Outtake() {
    
    TalonFXConfiguration pivotMMConfig = new TalonFXConfiguration();
    Slot0Configs pivotConfigs = new Slot0Configs();

    pivotConfigs.kP = 3;
    pivotConfigs.kI = 0;
    pivotConfigs.kD = 0;

    pivotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfigs.kG = 0.17;

    pivotMMConfig.CurrentLimits.SupplyCurrentLimit = 80;
    pivotMMConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotMMConfig.CurrentLimits.StatorCurrentLimit = 80;
    pivotMMConfig.CurrentLimits.StatorCurrentLimitEnable = true;


    pivotMMConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotMMConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotMMConfig.MotionMagic.MotionMagicAcceleration = 0.25;
    pivotMMConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
    pivotMMConfig.MotionMagic.MotionMagicJerk = 0.5;
    pivotMMConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMMConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotMMConfig.withSlot0(pivotConfigs);

    pivotMotor.getConfigurator().apply(pivotMMConfig);
  }

  @Override 
  public void periodic() {}

  public Command setPosition(double pos) {
    return new Command() {
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
        addRequirements(new Elevator());
      }
      
      @Override
      public void execute() {
        pivotMotor.setControl(new MotionMagicExpoVoltage(pos));
      }
    };
  }


  public Command runIntake() {
    return new Command() {
      @Override
      public boolean isFinished() {
          return outtakeBanner.getDistance().getValueAsDouble() < 10;
      }

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
        outtakeMotor.set(0.3);
      }
    };
  }


}
