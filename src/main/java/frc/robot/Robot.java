// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.Timestamp;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ButtonBoard.Action;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
    XboxController controller;
    ButtonBoard board;

    RobotContainer container;
    Command autonomousCommand;
    private final Field2d m_field = new Field2d();




    

   
            AutoChooser autoChooser;

             StructPublisher<Pose2d> robotPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Robot Pose", Pose2d.struct).publish();

    StructPublisher<Translation2d> translationPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Translation", Translation2d.struct).publish();

    StructPublisher<Transform2d> transform2d = NetworkTableInstance.getDefault()
.getStructTopic("MyStates", Transform2d.struct).publish();





    public Robot() {
        
        controller = new XboxController(Constants.controllerID);
        board = new ButtonBoard(Constants.boardID);

        container = new RobotContainer();
        CommandScheduler.getInstance().cancelAll();

        autoChooser = new AutoChooser();


    }

    @Override
    public void robotInit() {
        SmartDashboard.putData("Field", m_field);
        container.getDrivetrain().setCurrentPose(new Pose2d(7.1252,7.570, new Rotation2d()));
    }

    @Override
    public void robotPeriodic() {
        if(container.getVision().getEstimates().equals(container.getDrivetrain().getRobotPose())){
            container.setCorrectPosition();
        };
        m_field.setRobotPose(container.getDrivetrain().getRobotPose());

        SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);

          SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());

          SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());



        robotPublisher.set(container.getDrivetrain().getRobotPose());
        translationPublisher.set(container.getDrivetrain().geTranslation2d());
        

        CommandScheduler.getInstance().run();

        container.getDrivetrain().updateRobotHeight(container.getElevator().getPosition());
        

        container.updateLEDs();


        if (container.getMode() == RobotContainer.Mode.Coral) {
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


        container.getDrivetrain().addVisionMeasurement(container.getDrivetrain().getRobotPose(), Timer.getFPGATimestamp());


        autonomousCommand = container.getAutonomousCommand();

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
        CommandScheduler.getInstance().cancelAll();

         if (autonomousCommand != null) {
             autonomousCommand.schedule();
         }
        //Center_l4(container).schedule();
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
        if (controller.getXButtonPressed()){
         container.getDrivetrain().seedFieldCentric();
        } 

        
    

        container.driveJoysticks(
                        controller.getLeftX(),
                        controller.getLeftY(),
                        controller.getRightX(),
                        controller.getLeftTriggerAxis() > 0.2
                ).schedule();

        


        if (board.getButtonPressed(Action.Mode_Coral)) 
        {
            container.modeCoral();
            CommandScheduler.getInstance().cancelAll();
        }
        if (board.getButtonPressed(Action.Mode_Algae))
        {
            container.modeAlgae();
            CommandScheduler.getInstance().cancelAll();
        } 

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
        CommandScheduler.getInstance().cancelAll();

    }

    public static Command Center_l4(RobotContainer container){
        return Commands.sequence(
            container.getDrivetrain().driveSpeeds(new ChassisSpeeds(1,0,0)),
            Commands.waitSeconds(3),
            container.getDrivetrain().driveSpeeds(new ChassisSpeeds()),
            container.runAutoOuttake()
        );
    }
}
    