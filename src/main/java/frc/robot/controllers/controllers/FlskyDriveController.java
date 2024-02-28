// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.controllers.interfaces.DriverController;

/** Add your docs here. */
public class FlskyDriveController implements DriverController {
    
    private GenericHID controller;
    private JoystickButton fieldOrientedButton;
    private JoystickButton resetGyroButton;
    private JoystickButton reseteModuleAbsuluteButton;


    public FlskyDriveController(int id) {
        controller = new GenericHID(id);

        fieldOrientedButton = new JoystickButton(controller, 1);
        resetGyroButton = new JoystickButton(controller, 7);
        reseteModuleAbsuluteButton = new JoystickButton(controller, 5);
    
    }



    @Override
    public double getXSpeed() {
        return controller.getRawAxis(0) 
        * ((controller.getRawAxis(5) + 1) / 2);
    }

    @Override
    public double getYSpeed() {
        return controller.getRawAxis(1)
        * ((controller.getRawAxis(5) + 1) / 2);
    }

    @Override
    public double getRotationSpeed() {
        return controller.getRawAxis(3)
        * ((controller.getRawAxis(5) + 1) / 2);
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
    public JoystickButton getModulesToAbsuluteButton(){
        return reseteModuleAbsuluteButton;
    }
    @Override
    public double getShooterPotentiometer() {
        return 1;
    }




}
