// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class AutoRoutines {
        public static Command leave(Container container) {
                return Commands.sequence(
                        container.getDrivetrain().driveSpeeds(new ChassisSpeeds(-1, 0, 0)),
                        Commands.waitSeconds(5),
                        container.getDrivetrain().driveSpeeds(new ChassisSpeeds())
                );
        }
        public static Command intakeCoral(Container container){
         Command command = Commands.sequence(
                container.runIntake()
                )
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

                return command;
        }
}
