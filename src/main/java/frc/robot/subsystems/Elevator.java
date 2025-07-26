// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

public class Elevator extends SubsystemBase {
        TalonFX left, right;

        public enum Position {
                Low_Stow(0),
                High_Stow(2),

                L2_Coral(5),
                L3_Coral(11),
                L4_Coral(21),

                Low_Algae(10),
                High_Algae(16),
                End_Barge(23);

                public double value;

                Position(double value) {
                        this.value = value;
                }
        }

        public Elevator(int leftID, int rightID) {
                left = new TalonFX(leftID);
                right = new TalonFX(rightID);

                TalonFXConfiguration config = new TalonFXConfiguration();
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                config.Slot0.kP = 2;
                config.Slot0.kG = 0.6;

                config.MotionMagic.MotionMagicCruiseVelocity = 40;
                config.MotionMagic.MotionMagicAcceleration = 60;
                left.getConfigurator().apply(config);
        }

        public double getPosition() {
                return left.getPosition().getValueAsDouble();
        }

        public Command setPosition(Position position) {
                return new Command() {
                        public void execute() {
                                left.setControl(new MotionMagicExpoVoltage(position.value));
                                right.setControl(new Follower(left.getDeviceID(), false));
                        }
                        
                        public boolean isFinished() {
                                return Utilities.inTolerance(position.value - getPosition(), 0.4);
                        }
                };
        }

        @Override
        public void periodic() {}
}
