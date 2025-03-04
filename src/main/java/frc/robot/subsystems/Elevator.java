package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
<<<<<<< HEAD
=======

>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

<<<<<<< HEAD
  public TalonFX left = new TalonFX(21);
  public TalonFX right = new TalonFX(20);
=======
  public TalonFX left;
  public TalonFX right;
>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba


  public enum ElevatorMode {
    Test_1(0),
    Test_2(0),

    Stow(0),
    L2_Coral(0),
    L3_Coral(0),
    L4_Coral(0),
    L2_Algae(0),
    L3_Algae(0),
    Source(0),
    Processor(0);

    public final double pos;

    private ElevatorMode(double pos) {
        this.pos = pos;
    }
<<<<<<< HEAD
  }



  public Elevator() {
=======
  
  }

  public Elevator() {
    left = new TalonFX(Constants.elevatorLeftID);
    right = new TalonFX(Constants.elevatorRightID);
>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba

    right.setControl(new Follower(left.getDeviceID(), false));
    
    TalonFXConfiguration elevatorMMConfig = new TalonFXConfiguration();
    Slot0Configs elevatorConfigs = new Slot0Configs();

    elevatorConfigs.kP = 3;
    elevatorConfigs.kI = 0;
    elevatorConfigs.kD = 0;

    elevatorConfigs.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfigs.kG = 0.17;

    elevatorMMConfig.CurrentLimits.SupplyCurrentLimit = 80;
    elevatorMMConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorMMConfig.CurrentLimits.StatorCurrentLimit = 80;
    elevatorMMConfig.CurrentLimits.StatorCurrentLimitEnable = true;

<<<<<<< HEAD

    elevatorMMConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    elevatorMMConfig.MotionMagic.MotionMagicAcceleration = 100;
    elevatorMMConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
    elevatorMMConfig.MotionMagic.MotionMagicJerk = 10;
=======
    elevatorMMConfig.Feedback.FeedbackRemoteSensorID = left.getDeviceID();
    elevatorMMConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elevatorMMConfig.MotionMagic.MotionMagicAcceleration = 1;
    elevatorMMConfig.MotionMagic.MotionMagicCruiseVelocity = 0.7;
    elevatorMMConfig.MotionMagic.MotionMagicJerk = 0.1;
>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba
    elevatorMMConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMMConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMMConfig.withSlot0(elevatorConfigs);

    left.getConfigurator().apply(elevatorMMConfig);
    right.getConfigurator().apply(elevatorMMConfig);
<<<<<<< HEAD

=======
    
    right.setControl(new Follower(left.getDeviceID(), false));
>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba
  }

  @Override 
  public void periodic() {}

  public Command setPosition(ElevatorMode elevatorMode) {
    return new Command() {
      @Override
<<<<<<< HEAD
      public boolean isFinished() {
          return true;
=======
      public void execute() {
        left.setControl(new MotionMagicExpoVoltage(elevatorMode.pos));
>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba
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
<<<<<<< HEAD
      public void execute() {
        left.setControl(new MotionMagicExpoVoltage(elevatorMode.pos));
      }
    };
  }






}
=======
      public void end(boolean interupted){
        super.end(interupted);
      }
    };
  }
  };
>>>>>>> 0ceb695fbfda5c48a0deb04abde04b084d28f4ba
