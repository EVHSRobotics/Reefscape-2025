// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

public class Arm extends SubsystemBase {
        TalonFX pivot, rollers;
        CANrange coralRange, algaeRange;

        Timer timer;

        public enum Position {
                High_Stow(3.5),
                Low_Stow(21),

                Intake_Coral(-0.1),
                L1_Coral(14),
                L4_Coral(6),

                Intake_Algae(25),
                Start_Barge(27);

                public double value;

                Position(double value) {
                        this.value = value;
                }
        }

        public Arm(int pivotID, int rollersID, int coralRangeID, int algaeRangeID, String canID) {
                pivot = new TalonFX(pivotID, canID);
                rollers = new TalonFX(rollersID, canID);

                coralRange = new CANrange(coralRangeID, canID);
                algaeRange = new CANrange(algaeRangeID, canID);

                TalonFXConfiguration config = new TalonFXConfiguration();
                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                config.Slot0.kP = 10;

                config.MotionMagic.MotionMagicCruiseVelocity = 40;
                config.MotionMagic.MotionMagicAcceleration = 60;
                pivot.getConfigurator().apply(config);

                timer = new Timer();
                timer.start();
        }

        public double getPosition() {
                return pivot.getPosition().getValueAsDouble();
        }

        public boolean hasCoral() {
                return coralRange.getIsDetected(true).getValue();
        }

        public boolean hasAlgae() {
                return algaeRange.getIsDetected(true).getValue();
        }

        public Command setPosition(Position position) {
                return new Command() {
                        public void execute() {
                                pivot.setControl(new MotionMagicExpoVoltage(position.value));
                        }

                        public boolean isFinished() {
                                return Utilities.inTolerance(position.value - getPosition(), 0.2);
                        }
                };
        }

        public Command intakeCoral() {
                return new Command() {
                        public void execute() {
                                rollers.set(-0.6);
                        }

                        public boolean isFinished() {
                                return hasCoral();
                        }

                        public void end(boolean interrupted) {
                                rollers.set(0);
                        }
                };
        }

        public Command intakeAlgae() {
                return new Command() {
                        public void initialize() {
                                timer.reset();
                        }

                        public void execute() {
                                rollers.set(0.6);
                        }

                        public boolean isFinished() {
                                return hasAlgae();
                        }

                        public void end(boolean interrupted) {
                                rollers.set(0);
                        }
                };
        }

        public Command outtakeCoral(double power) {
                return new Command() {
                        public void initialize() {
                                timer.reset();
                        }

                        public void execute() {
                                rollers.set(power);
                        }

                        public boolean isFinished() {
                                return !hasCoral() && timer.get() > 0.5;
                        }

                        public void end(boolean interrupted) {
                                rollers.set(0);
                        }
                };
        }

        public Command outtakeAlgae(double power) {
                return new Command() {
                        public void initialize() {
                                timer.reset();
                        }

                        public void execute() {
                                rollers.set(power);
                        }

                        public boolean isFinished() {
                                return !hasAlgae() && timer.get() > 0.5;
                        }

                        public void end(boolean interrupted) {
                                rollers.set(0);
                        }
                };
        }

        @Override
        public void periodic() {
                if (hasAlgae() && !hasCoral()) rollers.set(0.2);
        }
}
