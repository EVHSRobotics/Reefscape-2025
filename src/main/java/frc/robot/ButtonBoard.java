// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot.ButtonMode;

/** Add your docs here. */
public class ButtonBoard {
    XboxController board;

    public ButtonBoard(int id) {
        board = new XboxController(id);
    }

    public boolean getButton(ButtonMode mode) {
        return board.getRawButton(mode.id);
    }

    public boolean getButtonPressed(ButtonMode mode) {
        return board.getRawButtonPressed(mode.id);
    }

    public boolean getButtonReleased(ButtonMode mode) {
        return board.getRawButtonReleased(mode.id);
    }
}
