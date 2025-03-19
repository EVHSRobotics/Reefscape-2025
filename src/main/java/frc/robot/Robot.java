// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Elevator;


public class Robot extends TimedRobot {
        XboxController controller;
        ButtonBoard board;

        Container container;

        Pose2d leftTarget, rightTarget, centerTarget;

                AutoChooser autoChooser;

        public Robot() {
                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);

                container = new Container();
                CommandScheduler.getInstance().cancelAll();
                autoChooser = new AutoChooser();
                
                autoChooser.addCmd("Intake Coral", () -> AutoRoutines.intakeCoral(container));


                
        }

        @Override
        public void robotPeriodic() {

                container.drivetrain.displayValues();
                CommandScheduler.getInstance().run();

                container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
                container.updateLEDs();

                int side = Utilities.getClosestSide(Constants.Vision.centerPoses, container.getDrivetrain().getRobotPose());

                leftTarget = Constants.Vision.leftPoses[side];
                rightTarget = Constants.Vision.rightPoses[side];
                centerTarget = Constants.Vision.centerPoses[side];


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

                SmartDashboard.updateValues();
        }

       

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

         public void autonomousInit() {
                Command autoCommand = container.getAutonomousCommand();
    if (autoCommand != null) {
        autoCommand.schedule();
    } else {
        // Fallback to simple leave routine if no auto command is available
        // AutoRoutines.leave(container).schedule();
    }
        }

        @Override
        public void teleopInit() {}

        @Override
        public void teleopPeriodic() {
                if (board.getButton(Action.Align_Left)) container.driveToPose(leftTarget).schedule();
                if (board.getButton(Action.Align_Right)) container.driveToPose(rightTarget).schedule();
                if (board.getButton(Action.Align_Center)) container.driveToPose(centerTarget).schedule();

                container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.2
                ).schedule();

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
        public void teleopExit() {}
}
