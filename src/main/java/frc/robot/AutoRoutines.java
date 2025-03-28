package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private Drivetrain m_Drivetrain;

    public AutoRoutines(AutoFactory factory, Drivetrain drivetrain) {
        m_factory = factory;
        m_Drivetrain = drivetrain;
    }


     public static Command left3Coral(Container container) {
                return Commands.sequence(
                        container.getDrivetrain().startTrajectory("testPath"),

                        container.getDrivetrain().followTrajectory("testPath")
  
                );
        }
}