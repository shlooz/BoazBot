package frc.robot.controllers.interfaces;


import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperationController {

    public Trigger getToSpeakerPositionButton();
    public Trigger getToAmpPositionButton();
    public Trigger getToGroundPositionButton();
    public Trigger getIntakeMenualSpeedButton();

    public Trigger getShootingButton();
    public Trigger getWarmingButton();

    public double getClimbingSpeed();
    public double getIntakeSpeed();

}
