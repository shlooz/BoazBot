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
        return  intakeFeederSubsystem.feedingCommand(-2)
            .alongWith(intakePositionSubsystem.moveToAngle(GROUND_INTAKE_ANGLE))
            .alongWith(intakeFeederSubsystem.feedingCommand(-1))
            .andThen(new WaitUntilCommand(endIntakeTrigger))
            .andThen(closeIntake());
    }

    public Command shootSpeaker(){
        return warmShooter()
                .andThen(new WaitUntilCommand(endIntakeTrigger))
                .andThen(closeIntake())
                .andThen(releaseToShooter())
                .andThen(shooterSubsystem.shootingCommand());
    }

    public Command ampIntake(){
        return intakePositionSubsystem.moveToAngle(SPEAKER_INTAKE_ANGLE)
            .alongWith(intakeFeederSubsystem.feedingCommand(0))
            .andThen(new WaitUntilCommand(endIntakeTrigger))
            .andThen(closeIntake());
    }

    public Command closeIntake(){
        return intakePositionSubsystem.moveToAngle(SPEAKER_INTAKE_ANGLE)
        .alongWith(intakeFeederSubsystem.feedingCommand())
        .raceWith(new WaitCommand(10));
    }
    
    public Command warmShooter(){
        return shooterSubsystem.shootingCommand(LEFT_MOTOR_SPEED_SPEAKER, RIGHT_MOTOR_SPEED_SPEAKER);
    }
    
    public Command closeShooter(){
        return shooterSubsystem.shootingCommand(0, 0)
    }
    

    public Command releaseToShooter(){
        return intakeFeederSubsystem.feedingCommand(0.2)
                .andThen(new WaitCommand(3))
                .andThen(intakeFeederSubsystem.feedingCommand(0));
    }

    public Command shootIntake(double speed){
        return intakeFeederSubsystem.feedingCommand(speed);
    }

    public Command moveIntakeManualy(double speed){
        System.out.println("angle motor speed is equal to" + speed);
        return intakePositionSubsystem.runAngleMotorCommand(
            intakePositionSubsystem.getIntakePosition() == 0 ? speed * 0.5 : 0);
    }


}
