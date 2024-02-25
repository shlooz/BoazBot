// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFeederSubsystem extends SubsystemBase {
  /** Creates a new IntakeFeederSubsystem. */
  
  private final CANSparkMax feedingMotor;

  private double Qxpotentiometer;

  public IntakeFeederSubsystem() {
    feedingMotor = new CANSparkMax(FEEDING_MOTOR_ID, MotorType.kBrushless);
    feedingMotor.restoreFactoryDefaults();
    feedingMotor.setIdleMode(IdleMode.kCoast);//#endregion

    Qxpotentiometer = 0;

  }

  @Override
  public void periodic() {
    // System.out.println(Qxpotentiometer);
    // This method will be called once per scheduler run
  }

  public void feeding(double speed){
    Qxpotentiometer = (speed * 2) - 1;
    feedingMotor.set(speed);
  }

  public Command feedingCommand(double speed){
    return new RunCommand(() -> feeding(speed));
  }

  public Command feedingCommand(){
    return feedingCommand(0);
  }

}
