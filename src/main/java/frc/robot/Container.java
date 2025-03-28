// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;

        CANdle lights;

        Mode mode;
        Elevator.Position coralLevel, algaeLevel;

        private final SendableChooser<String> autoChooser;
        private final HashMap<String, Command> commandChooser;

        public enum Mode {
                Coral,
                Algae
        }

        public enum AutoPaths {
                // Define your Reefscape auto paths
                LeftCoral("Left_3_coral");

                private String pathName;

                AutoPaths(String pathName) {
                        this.pathName = pathName;
                }

                public String getPath() {
                        return this.pathName;
                }
        }

        public Container() {



                drivetrain = new Drivetrain(
                                Constants.Drivetrain.drivetrainConfigs,

                                Constants.Drivetrain.frontLeftConfigs,
                                Constants.Drivetrain.frontRightConfigs,
                                Constants.Drivetrain.backLeftConfigs,
                                Constants.Drivetrain.backRightConfigs);

                arm = new Arm(Constants.Arm.pivotID, Constants.Arm.rollersID, Constants.Arm.coralRangeID,
                                Constants.Arm.algaeRangeID);
                elevator = new Elevator(Constants.Elevator.leftID, Constants.Elevator.rightID, Constants.canivoreID);

                // vision = new Vision(
                // Constants.Vision.frontID,

                // Constants.Vision.frontLeftID, Constants.Vision.frontLeftOffset,
                // Constants.Vision.frontRightID, Constants.Vision.frontRightOffset,
                // Constants.Vision.backLeftID, Constants.Vision.backLeftOffset,
                // Constants.Vision.backRightID, Constants.Vision.backRightOffset
                // );

                lights = new CANdle(Constants.lightsID);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;

                autoChooser = new SendableChooser<String>();
    commandChooser = new HashMap<String, Command>();
    
    // Add auto options
    AutoPaths[] allAutoPaths = AutoPaths.values();
    for (int i=0; i<allAutoPaths.length; i++) {
        commandChooser.put(allAutoPaths[i].getPath(), drivetrain.getAutoPath(allAutoPaths[i].getPath()));
        autoChooser.addOption(allAutoPaths[i].getPath(), allAutoPaths[i].getPath());
    }
    
    SmartDashboard.putData(autoChooser);
    
    // Set up auto commands
    setUpAutoCommands();

                setupAutoBindings();
        }

        private void setupAutoBindings() {
                drivetrain.bindCommandsAuto("intake", runIntake());
        }

        public void setUpAutoCommands() {
                HashMap<String, Command> eventMap = new HashMap<String, Command>();

                eventMap.put("intakeCoral", runCoralIntake());

                NamedCommands.registerCommands(eventMap);
        }

        public Drivetrain getDrivetrain() {
                return drivetrain;
        }

        // In your Container.java class constructor or a setup method

        public Arm getArm() {
                return arm;
        }

        public Elevator getElevator() {
                return elevator;
        }

        public Mode getMode() {
                return mode;
        }

        public Elevator.Position getCoralLevel() {
                return coralLevel;
        }

        public Elevator.Position getAlgaeLevel() {
                return algaeLevel;
        }

        public void updateLEDs() {
                if (mode == Mode.Coral)
                        lights.setLEDs(255, 0, 255);
                else
                        lights.setLEDs(0, 255, 0);
        }

        public void modeCoral() {
                mode = Mode.Coral;
        }

        public void modeAlgae() {
                mode = Mode.Algae;
        }

        public void targetLow() {
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public void targetMedium() {
                coralLevel = Elevator.Position.L3_Coral;
        }

        public void targetHigh() {
                coralLevel = Elevator.Position.L4_Coral;
                algaeLevel = Elevator.Position.High_Algae;
        }

        public Command driveJoysticks(double leftX, double leftY, double rightX, boolean slowed) {
                ChassisSpeeds speeds = new ChassisSpeeds(
                                -leftY * Constants.Drivetrain.maxSpeed,
                                -leftX * Constants.Drivetrain.maxSpeed,
                                -rightX * Constants.Drivetrain.maxAngularSpeed);
                Command command = drivetrain.driveSpeeds(speeds, slowed)

                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(drivetrain);

                return command;
        }

        public Command driveToPose(Pose2d targetPose) {
                Pose2d robotPose = drivetrain.getRobotPose();

                ChassisSpeeds speeds = new ChassisSpeeds(
                                Constants.Drivetrain.translationPID.calculate(robotPose.getX(), targetPose.getX()),
                                Constants.Drivetrain.translationPID.calculate(robotPose.getY(), targetPose.getY()),
                                Constants.Drivetrain.headingPID.calculate(Utilities.getRadians(robotPose),
                                                Utilities.getRadians(targetPose)));
                Command command = drivetrain.driveSpeeds(speeds)

                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                command.addRequirements(drivetrain);

                return command;
        }

        public Command stow() {
                Command command = Commands.sequence(
                                Commands.either(
                                                arm.setPosition(Arm.Position.Processor),
                                                arm.setPosition(Arm.Position.Stow),

                                                () -> arm.hasAlgae()),
                                elevator.setPosition(Elevator.Position.Stow))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runCoralIntake() {
                Command command = Commands.sequence(
                                arm.setPosition(Arm.Position.Stow),
                                elevator.setPosition(Elevator.Position.Stow),
                                Commands.parallel(
                                                arm.setPosition(Arm.Position.Intake_Coral),
                                                arm.intakeCoral()),
                                arm.setPosition(Arm.Position.Stow)

                )
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runIntake() {
                Command command = Commands.sequence(
                                arm.setPosition(Arm.Position.Stow),
                                Commands.either(
                                                Commands.sequence(
                                                                elevator.setPosition(Elevator.Position.Stow),
                                                                Commands.parallel(
                                                                                arm.setPosition(Arm.Position.Intake_Coral),
                                                                                arm.intakeCoral()),
                                                                arm.setPosition(Arm.Position.Stow)),
                                                Commands.parallel(
                                                                arm.setPosition(Arm.Position.Intake_Algae),
                                                                elevator.setPosition(algaeLevel),
                                                                arm.intakeAlgae()),

                                                () -> mode == Mode.Coral))
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command runOuttake() {
                Command command = Commands.either(
                                Commands.either(
                                                arm.outtakeCoral(),
                                                Commands.sequence(
                                                                arm.setPosition(Arm.Position.Stow),
                                                                elevator.setPosition(coralLevel),
                                                                arm.setPosition(coralLevel == Elevator.Position.L4_Coral
                                                                                ? Arm.Position.L4_Coral
                                                                                : Arm.Position.Stow)),

                                                () -> Utilities.inTolerance(coralLevel.value - elevator.getPosition(),
                                                                0.4)),
                                Commands.either(
                                                Commands.parallel(
                                                                elevator.setPosition(Elevator.Position.End_Barge),
                                                                arm.setPosition(Arm.Position.Stow),
                                                                Commands.sequence(
                                                                                Commands.waitSeconds(0.5),
                                                                                arm.outtakeAlgae(-1))),
                                                Commands.either(
                                                                arm.outtakeAlgae(-0.4),
                                                                Commands.sequence(
                                                                                arm.setPosition(Arm.Position.Processor),
                                                                                elevator.setPosition(
                                                                                                Elevator.Position.High_Algae),
                                                                                arm.setPosition(Arm.Position.Start_Barge)),

                                                                () -> Utilities.inTolerance(Elevator.Position.Stow.value
                                                                                - elevator.getPosition(), 0.4)),

                                                () -> Utilities.inTolerance(Elevator.Position.High_Algae.value
                                                                - elevator.getPosition(), 0.4)),

                                () -> mode == Mode.Coral)
                                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command climb() {
                Command command = Commands.either(
                                Commands.sequence(
                                                elevator.setPosition(Elevator.Position.Stow)),
                                Commands.sequence(
                                                arm.setPosition(Arm.Position.Stow),
                                                elevator.setPosition(Elevator.Position.Start_Climb)),

                                () -> Utilities.inTolerance(
                                                Elevator.Position.Start_Climb.value - elevator.getPosition(), 0.4))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
                command.addRequirements(arm, elevator);

                return command;
        }

        public Command getAutonomousCommand() {
                return commandChooser.get(autoChooser.getSelected());
            }
}
