// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controllers.interfaces.DriverController;

/** Add your docs here. */
public class QXDriveController implements DriverController {
    
    private GenericHID controller;
    private JoystickButton fieldOrientedButton;
    private JoystickButton resetGyroButton;


    public QXDriveController(int id) {
        controller = new GenericHID(id);

        fieldOrientedButton = new JoystickButton(controller, 10);
        resetGyroButton = new JoystickButton(controller, 3);
    
    }

    private double getDriveFactor() {
        return ((controller.getRawAxis(4) + 1) / 2) *
                ((controller.getRawAxis(4) + 1) / 2);
    }
    
    @Override
    public double getXSpeed() {
        return - controller.getRawAxis(2) * getDriveFactor();
    }

    @Override
    public double getYSpeed() {
        return controller.getRawAxis(3) * getDriveFactor();
    }

    @Override
    public double getRotationSpeed() {
        return controller.getRawAxis(0) * getDriveFactor();
    }

    @Override
    public boolean getFieldOriented() {
        return fieldOrientedButton.getAsBoolean();
    }

    @Override
    public JoystickButton getResetGyroButton() {
        return resetGyroButton;
    }
    // @Override
    // public JoystickButton getModulesToAbsuluteButton(){
    //     return reseteModuleAbsuluteButton;
    // }

    @Override
    public double getShooterPotentiometer() {
        return controller.getRawAxis(5);
    }




}
