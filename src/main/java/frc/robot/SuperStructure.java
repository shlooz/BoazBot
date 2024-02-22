package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;


public class SuperStructure {
    public final IntakePositionSubsystem intakePositionSubsystem;
    public final IntakeFeederSubsystem intakeFeederSubsystem;
    public final ShooterSubsystem shooterSubsystem;

    public final Trigger endIntakeTrigger;

    public SuperStructure(Trigger endIntakeTrigger){
        intakePositionSubsystem = new IntakePositionSubsystem();
        intakeFeederSubsystem = new IntakeFeederSubsystem();
        shooterSubsystem = new ShooterSubsystem();   

        this.endIntakeTrigger = endIntakeTrigger;
        
    }

    public Command groundIntake(){
        return intakePositionSubsystem.moveToAngle(GROUND_INTAKE_ANGLE)
            .alongWith(intakeFeederSubsystem.feedingCommand(-0.2))
            .andThen(new WaitUntilCommand(endIntakeTrigger))
            .andThen(closeIntake());
    }

    public Command shootSpeaker(){
        return warmShooter()
                .andThen(new WaitUntilCommand(endIntakeTrigger))
                .andThen(closeIntake())
                .andThen(releaseToShooter())
                .andThen(closeShooter());
    }

    public Command ampIntake(){
        return intakePositionSubsystem.moveToAngle(SPEAKER_INTAKE_ANGLE)
            .alongWith(intakeFeederSubsystem.feedingCommand(0))
            .andThen(new WaitUntilCommand(endIntakeTrigger))
            .andThen(closeIntake());
    }

    public Command closeIntake(){
        return intakePositionSubsystem.moveToAngle(SPEAKER_INTAKE_ANGLE)
        .alongWith(intakeFeederSubsystem.feedingCommand(0));
    }
    
    public Command warmShooter(){
        return shooterSubsystem.shootingCommand(LEFT_MOTOR_SPEED_SPEAKER, RIGHT_MOTOR_SPEED_SPEAKER)
            .withTimeout(5)
            .andThen(closeShooter());
    }
    
    
    public Command closeShooter(){
        return shooterSubsystem.shootingCommand();
    }

    public Command releaseToShooter(){
        return intakeFeederSubsystem.feedingCommand(0.2)
                .andThen(new WaitCommand(3))
                .andThen(intakeFeederSubsystem.feedingCommand(0));
    }

    public Command shootIntake(double speed){
        return intakeFeederSubsystem.feedingCommand(speed).
            withTimeout(0.5).
                andThen(closeIntake()).
                alongWith(warmShooter());
    }



}
