// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controllers.interfaces.PrimaryDriverController;

/** Add your docs here. */
public class QXDriveController implements PrimaryDriverController {

    private GenericHID controller;
    private JoystickButton fieldOrientedButton;
    private JoystickButton resetGyroButton;
    private JoystickButton yeetingButton;
    private JoystickButton snapButton;
    private JoystickButton balancingButton;
    private JoystickButton reverseButton;

    public QXDriveController(int id) {
        controller = new GenericHID(id);
        fieldOrientedButton = new JoystickButton(controller, 1);
        resetGyroButton = new JoystickButton(controller, 2);
        balancingButton = new JoystickButton(controller, 3);
        yeetingButton = new JoystickButton(controller, 4);
        snapButton = new JoystickButton(controller, 5);
        reverseButton = new JoystickButton(controller, 6);
    
    }

    public static double deadband(double value, double lowerLimit){
        if (Math.abs(value) < lowerLimit) {
            return 0;
        } else {

            if (value > 0) {
                return (value - lowerLimit) / (1 - lowerLimit);
            } else {
                return (value + lowerLimit) / (1 - lowerLimit);
            }
        }
    }

    @Override
    public double getXSpeed() {
        return deadband(controller.getRawAxis(2), 0.05);
    }

    @Override
    public double getYSpeed() {
        return deadband(controller.getRawAxis(1), 0.02);
    }

    @Override
    public double getRotationSpeed() {
        return deadband(controller.getRawAxis(3), 0.02);
    }

    @Override
    public boolean getFieldOriented() {
        return fieldOrientedButton.getAsBoolean();
    }

    @Override
    public JoystickButton getResetGyroButton() {
        return resetGyroButton;
    }

    @Override
    public JoystickButton getYeetingButton() {
        return yeetingButton;
    }

    @Override
    public JoystickButton getSnapButton() {
        return snapButton;
    }

    @Override
    public JoystickButton getBalancingButton() {
        return balancingButton;
    }

    @Override
    public JoystickButton getReversingButton() {
        return reverseButton;
    }



}
