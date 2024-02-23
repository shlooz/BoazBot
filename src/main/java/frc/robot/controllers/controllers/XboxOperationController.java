// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class XboxOperationController {
    public GenericHID controller;

    public JoystickButton groundIntakeButton;
    public JoystickButton ampIntakeButton;
    public JoystickButton speakerIntakeButton;
    public JoystickButton manualIntakeButton;

    public JoystickButton warmingShooterButton;
    public JoystickButton shootingButton;

    public XboxOperationController(int ID){
        controller = new GenericHID(ID);

        groundIntakeButton = new JoystickButton(controller, 1);
        ampIntakeButton = new JoystickButton(controller, 2);
        manualIntakeButton = new JoystickButton(controller, 3);
        speakerIntakeButton = new JoystickButton(controller, 4);

        warmingShooterButton = new JoystickButton(controller, 5);
        shootingButton = new JoystickButton(controller, 6);
    }

    public Trigger getClosedPositionButton(){
        return groundIntakeButton;
    }
    public Trigger getToAmpPositionButton(){
        return ampIntakeButton;
    }
    public Trigger getToGroundPositionButton(){
        return speakerIntakeButton;
    }
    public Trigger getManualIntakeButton(){
        return manualIntakeButton;
    }

    public Trigger getShootingButton(){
        return shootingButton;
    }
    public Trigger getWarmingButton(){
        return warmingShooterButton;
    }

    public double getClimbingSpeed(){
        return controller.getRawAxis(1);
    }
    public double getIntakeSpeed(){
        System.out.println("button pressed");
        return controller.getRawAxis(5);
    }
}

