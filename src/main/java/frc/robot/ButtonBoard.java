// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class ButtonBoard {
        XboxController controller;

        public enum Action {
                Mode_Coral(1),
                Mode_Algae(2),

                Target_Low(3),
                Target_Medium(4),
                Target_High(5),
                Target_Trough(6),

                Reset(8);

                public int value;

                Action(int value) {
                        this.value = value;
                }
        }

        public ButtonBoard(int port) {
                controller = new XboxController(port);
        }

        public boolean getButtonPressed(Action action) {
                return controller.getRawButtonPressed(action.value);
        }

        public boolean getButtonReleased(Action action) {
                return controller.getRawButtonReleased(action.value);
        }

        public boolean getButton(Action action) {
                return controller.getRawButton(action.value);
        }
}
