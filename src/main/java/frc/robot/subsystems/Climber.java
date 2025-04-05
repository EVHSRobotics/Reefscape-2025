// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

public class Climber extends SubsystemBase {
        TalonFX winch;

        public enum Position {
                Stow(-60),
                Deploy(10);

                public double value;
                
                Position(double value) {
                        this.value = value;
                }
        }

        public Climber(int winchID, String canID) {
                winch = new TalonFX(winchID, canID);

                TalonFXConfiguration config = new TalonFXConfiguration();
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                config.Slot0.kP = 10;

                config.MotionMagic.MotionMagicCruiseVelocity = 20;
                config.MotionMagic.MotionMagicAcceleration = 30;
                winch.getConfigurator().apply(config);
        }

        public double getPosition() {
                return winch.getPosition().getValueAsDouble();
        }

        public Command setPosition(Position position) {
                return new Command() {
                        public void execute() {
                                winch.setControl(new MotionMagicExpoVoltage(position.value));
                        }

                        public boolean isFinished() {
                                return Utilities.inTolerance(position.value - getPosition(), 0.2);
                        }
                };
        }

        @Override
        public void periodic() {}
}