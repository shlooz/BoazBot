package frc.robot.controllers.controllers;

import static frc.robot.Constants.ControllerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */

public class QXOperationController {
    private GenericHID controller;

    private Trigger groundIntakeButton;
    private Trigger ampIntakeButton;
    private Trigger closeIntakeButton;
    private Trigger stopIntakeButton;

    private Trigger manualIntakeDownButton;
    private Trigger manualIntakeUpButton;

    private Trigger intakingButton;
    private Trigger outakingButton;

    private Trigger climbingButton;

    private Trigger warmingShooterButton;
    private Trigger shootingButton;

    public QXOperationController(int ID){

        
        controller = new GenericHID(ID);

        groundIntakeButton = new Trigger(() -> isIntakeDown());
        ampIntakeButton = new Trigger(() -> isIntakeMid());
        closeIntakeButton = new Trigger(() -> isIntakeUp());
        stopIntakeButton = new JoystickButton(controller, 10);

        warmingShooterButton = new Trigger(() -> warmShootingButton());
        shootingButton = new Trigger(() -> warmShootingButton());

        manualIntakeDownButton = new Trigger(() -> downArrow());
        manualIntakeUpButton = new Trigger(() -> upArrow());

        intakingButton = new JoystickButton(controller, 4);
        outakingButton = new JoystickButton(controller, 2);

        climbingButton = new Trigger(() -> isClimbingOn());
    }


    private boolean isIntakeDown(){
        return controller.getRawAxis(7) == 1;
    }
    private boolean isIntakeMid(){
        return controller.getRawAxis(7) == 0;
    }
    private boolean isIntakeUp(){
        return controller.getRawAxis(7) == -1;
    }
    private boolean warmShootingButton(){
        return controller.getRawAxis(6) == -1;
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
        return MathUtil.applyDeadband(controller.getRawAxis(1), STICK_DEADBAND) 
            *  Math.abs(MathUtil.applyDeadband(controller.getRawAxis(1), STICK_DEADBAND)) ;
    }

    public double getIntakeSpeed(){
        return controller.getRawAxis(5);
    }
}

