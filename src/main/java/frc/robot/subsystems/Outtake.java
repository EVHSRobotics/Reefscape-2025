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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {

  public TalonFX outtakeMotor;
  public TalonFX outtakeRollers;
  public int outtakeMotorID;
  public int OuttakeRollersID;

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

  public Command setPosition(outtakeMode outtakeMode) {
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
      public void end(boolean interupted){

      }

  };

}

public Command[] runIntake() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runIntake'");
}
}
