// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision.Camera;

public class Robot extends TimedRobot {
    XboxController controller;
    ButtonBoard board;

    Container container;
    Command autonomousCommand;

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().
            getStructTopic("Robot Pose", Pose2d.struct).publish();         

    public Robot() {
        controller = new XboxController(Constants.controllerID);
        board = new ButtonBoard(Constants.boardID);

        container = new Container();
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void robotInit() {
        // Additional initialization if needed
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
        container.updateRobotPose(container.getVision().getEstimate(Camera.Front));

        container.updateLEDs();

        publisher.set(container.getRobotPose());

        if (container.getMode() == Container.Mode.Coral) {
            SmartDashboard.putBoolean("Low", container.getCoralLevel() == Elevator.Position.L2_Coral);
            SmartDashboard.putBoolean("Medium", container.getCoralLevel() == Elevator.Position.L3_Coral);
            SmartDashboard.putBoolean("High", container.getCoralLevel() == Elevator.Position.L4_Coral);
        }
        else {
            SmartDashboard.putBoolean("Low", container.getAlgaeLevel() == Elevator.Position.Low_Algae);
            SmartDashboard.putBoolean("High", container.getAlgaeLevel() == Elevator.Position.High_Algae);

            SmartDashboard.putBoolean("Medium", false);
        }

        SmartDashboard.putBoolean("Has Coral", container.getArm().hasCoral());
        SmartDashboard.putBoolean("Has Algae", container.getArm().hasAlgae());

        // Display relative pose
        Pose2d relativePose = container.getDrivetrain().getRelativePose();
        SmartDashboard.putNumber("Relative X", relativePose.getX());
        SmartDashboard.putNumber("Relative Y", relativePose.getY());
        SmartDashboard.putNumber("Relative Angle", relativePose.getRotation().getDegrees());

        SmartDashboard.updateValues();
    }

    @Override
    public void disabledInit() {
        // Reset any state when disabled
    }

    @Override
    public void disabledPeriodic() {
        // Check for any changes in auto selection
    }

    @Override
    public void autonomousInit() {
        // Initialize the relative pose system
        container.getDrivetrain().initializeRelativePose();
        
        // Get selected autonomous command
        autonomousCommand = container.getAutonomousCommand();
        
        // Schedule the autonomous command
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        } else {
            // Fallback to simple leave routine if no auto command is available
            AutoRoutines.leave(container).schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Autonomous periodic code (if needed beyond CommandScheduler)
    }

    @Override
    public void autonomousExit() {
        // Cancel the autonomous command when exiting autonomous
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        // Cancel autonomous command if it's still running
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    @Override
    public void teleopPeriodic() {
        container.driveJoysticks(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), controller.getLeftTriggerAxis() > 0.2);
        if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();

        if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral();
        if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae();

        if (controller.getAButtonPressed()) container.stow().schedule();
        if (controller.getYButtonPressed()) container.climb().schedule();
        
        if (board.getButtonPressed(Action.Target_Low)) container.targetLow();
        if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium();
        if (board.getButtonPressed(Action.Target_High)) container.targetHigh();

        if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
        if (controller.getRightBumperButtonPressed()) container.runOuttake().schedule();
    }

    @Override
    public void teleopExit() {
        // Cleanup when exiting teleop
    }
    
    @Override
    public void testInit() {
        // Cancel all running commands
        CommandSchedul