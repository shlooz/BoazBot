package frc.robot.controllers.interfaces;


import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperationDriverController {

    public Trigger getCubeButton();
    public Trigger getConeButton();
    
    public Trigger getGroundIntakeButton(); 
    public Trigger getStationIntakeButton();
    public Trigger getFlippedIntakeButton();

    public Trigger getTopScoringButton();
    public Trigger getMidScoringButton();
    public Trigger getLowScoringButton();
    public Trigger getYeetButton();

    public Trigger getOuttakeButton();
    public Trigger getFoldButton();

    public Trigger getZeroButton();

    public Trigger getPrintscreen();

    public double getAngleControl();
    public double getLengthControl();

}
