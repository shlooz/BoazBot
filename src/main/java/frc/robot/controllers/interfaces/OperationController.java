package frc.robot.controllers.interfaces;


import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperationController {

    public Trigger getToClosedPositionButton();
    public Trigger getToAmpPositionButton();
    public Trigger getToGroundPositionButton();

    public Trigger getShootingButton();
    public Trigger getWarmingButton();

    public double getClimbingSpeed();
    public double getIntakeSpeed();

    public Trigger getIsClimbing();

    public Trigger getManualIntakePositionDown();
    public Trigger getManualIntakePositionUp();

    public Trigger getIntakeIntakingButton();
    public Trigger getIntakeOutakingButton();



}
