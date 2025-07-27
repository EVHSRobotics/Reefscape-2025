// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Vision.Camera;

public class Robot extends TimedRobot {
        CommandScheduler scheduler;
        Timer timer;

        XboxController controller;
        XboxController controller1;
        ButtonBoard board;
        Container container;

        Pose2d robotPose = new Pose2d(), leftTarget = new Pose2d(), rightTarget = new Pose2d();
        StructPublisher<Pose2d> robotPublisher, leftPublisher, rightPublisher, QuestPublisher;

        Command routine;

        public Robot() {
                scheduler = CommandScheduler.getInstance();
                timer = new Timer();

                controller = new XboxController(Constants.controllerID);
                controller1 = new XboxController(1);
                board = new ButtonBoard(Constants.boardID);
                container = new Container();

                robotPublisher = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct)
                                .publish();
                QuestPublisher = NetworkTableInstance.getDefault().getStructTopic("Quest Pose", Pose2d.struct)
                                .publish();     

                leftPublisher = NetworkTableInstance.getDefault().getStructTopic("Left Target", Pose2d.struct)
                                .publish();
                rightPublisher = NetworkTableInstance.getDefault().getStructTopic("Right Target", Pose2d.struct)
                                .publish();

                NamedCommands.registerCommand("Intake", container.intake());
                NamedCommands.registerCommand("Outtake", container.autoOuttake());
                NamedCommands.registerCommand("Stow", container.stow());
        }

        @Override
        public void robotPeriodic() {

                SmartDashboard.putNumber("elevator position", container.elevator.getPosition());
                SmartDashboard.putNumber("arm position", container.arm.getPosition());
                SmartDashboard.putNumber(
                                "CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);
                SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
                SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
                SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
                SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

                scheduler.run();

                SmartDashboard.putData(CommandScheduler.getInstance());
                SmartDashboard.updateValues();

                container.updateRobotPose(container.vision.getEstimate(Camera.Front));
                if (container.drivetrain.questNav.getBatteryPercent() > 0)

                {
                        QuestPublisher.set(container.drivetrain.questNav.getRobotPose());
                        container.updateRobotPose(container.drivetrain.questNav.getRobotPose());
                }
                container.drivetrain.updateRobotHeight(container.elevator.getPosition());
                container.updateLEDs();

                robotPose = container.drivetrain.getRobotPose();
                robotPublisher.set(robotPose);

                int side = Utilities.getClosestSide(robotPose,
                                Constants.centerTargets);
                leftTarget = Constants.leftTargets[side];
                rightTarget = Constants.rightTargets[side];

                leftPublisher.set(leftTarget);
                rightPublisher.set(rightTarget);
        }

        @Override
        public void autonomousInit() {
                scheduler.cancelAll();

                container.drivetrain.seedFieldCentric();

                routine = Commands.sequence(
                        container.drivetrain.driveSpeeds(new ChassisSpeeds(3,0,0)),
                        Commands.waitSeconds(3),
                        container.drivetrain.driveSpeeds(new ChassisSpeeds(0,0,0)),
                        container.autoOuttake(),
                        container.stow());


                 routine.schedule();
        }

        @Override
        public void teleopInit() {
                scheduler.cancelAll();
        }

        @Override
        public void teleopPeriodic() {
                if (board.getButtonPressed(Action.Reset)) scheduler.cancelAll();

                container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.1
                ).schedule();

                if (controller.getPOV() == 270) container.scheduleOnly(container.alignToPose(leftTarget));
                if (controller.getPOV() == 90) container.scheduleOnly(container.alignToPose(rightTarget));
                
                if (controller.getLeftBumperButtonPressed()) container.scheduleOnly(container.intake());
                if (controller.getRightBumperButtonPressed()) container.scheduleOnly(container.teleOuttake());

                if (controller.getAButtonPressed()) container.scheduleOnly(container.stow());
                if (controller.getYButtonPressed()) container.climb().schedule();

                if (controller.getXButtonPressed()) container.drivetrain.seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.mode = Container.Mode.Coral;
                if (board.getButtonPressed(Action.Mode_Algae)) container.mode = Container.Mode.Algae;
                
                if (board.getButtonPressed(Action.Target_Low)) container.targetLow();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh();
                if (board.getButtonPressed(Action.Target_Trough)) container.targetTrough();
        }
}