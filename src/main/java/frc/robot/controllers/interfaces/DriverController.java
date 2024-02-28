package frc.robot.controllers.interfaces;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface DriverController {
    public double getXSpeed();

    public double getYSpeed();

    public double getRotationSpeed();

    public boolean getFieldOriented();

    public JoystickButton getResetGyroButton();

    public double getShooterPotentiometer();

    public JoystickButton getModulesToAbsuluteButton();
}
