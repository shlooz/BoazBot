// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers.controllers;

import static frc.robot.Constants.STICK_DEADBAND;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class XboxOperationController {
    private GenericHID controller;

    private JoystickButton groundIntakeButton;
    private JoystickButton ampIntakeButton;
    private JoystickButton closeIntakeButton;
    private JoystickButton stopIntakeButton;

    private Trigger manualIntakeDownButton;
    private Trigger manualIntakeUpButton;

    private Trigger intakingButton;
    private Trigger outakingButton;

    private Trigger climbingButton;

    private JoystickButton warmingShooterButton;
    private JoystickButton shootingButton;

    public XboxOperationController(int ID){
        controller = new GenericHID(ID);

        groundIntakeButton = new JoystickButton(controller, 1);
        ampIntakeButton = new JoystickButton(controller, 2);
        stopIntakeButton = new JoystickButton(controller, 3);
        closeIntakeButton = new JoystickButton(controller, 4);

        warmingShooterButton = new JoystickButton(controller, 6);
        shootingButton = new JoystickButton(controller, 7);

        manualIntakeDownButton = new Trigger(() -> downArrow());
        manualIntakeUpButton = new Trigger(() -> upArrow());

        intakingButton = new Trigger(() -> leftArrow());
        outakingButton = new Trigger(() -> rightArrow());

        climbingButton = new Trigger(() -> isClimbingOn());
    }

    private boolean upArrow(){
        return controller.getPOV() == 0;
    }
    private boolean rightArrow(){
        return controller.getPOV() == 90;
    }
    private boolean downArrow(){
        return controller.getPOV() == 180;
    }
    private boolean leftArrow(){
        return controller.getPOV() == 270;
    }
    
    private boolean isClimbingOn(){
        return MathUtil.applyDeadband(controller.getRawAxis(1), STICK_DEADBAND) != 0;
    }

    public Trigger getClosedPositionButton(){
        return closeIntakeButton;
    }
    public Trigger getToAmpPositionButton(){
        return ampIntakeButton;
    }
    public Trigger getToGroundPositionButton(){
        return groundIntakeButton;
    }
    public Trigger getStopIntakeButton(){
        return stopIntakeButton;
    }

    public Trigger getManualDownIntakeButton(){
        return manualIntakeDownButton;
    }
    public Trigger getManualUpIntakeButton(){
        return manualIntakeUpButton;
    }
    public Trigger getIntakeIntakingButton(){
        return intakingButton;
    }
    public Trigger getIntakeOutakingButton(){
        return outakingButton;
    }


    public Trigger getSpeakerShootingButton(){
        return shootingButton;
    }
    public Trigger getWarmingButton(){
        return warmingShooterButton;
    }
    
    public Trigger getClimbing(){
        return climbingButton;
    }
    public double getClimbingSpeed(){
        return MathUtil.applyDeadband(controller.getRawAxis(1), STICK_DEADBAND);
    }

    public double getIntakeSpeed(){
        // System.out.println("button pressed");
        return controller.getRawAxis(5);
    }
}

