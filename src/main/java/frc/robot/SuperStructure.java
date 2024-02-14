package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class SuperStructure {
    public final IntakePositionSubsystem intakePositionSubsystem;
    public final IntakeFeederSubsystem intakeFeederSubsystem;
    public final ShooterSubsystem shooterSubsystem;

    public SuperStructure(){
        intakePositionSubsystem = new IntakePositionSubsystem();
        intakeFeederSubsystem = new IntakeFeederSubsystem();
        shooterSubsystem = new ShooterSubsystem();   
    }

    public Command groundIntake(){
        return intakeFeederSubsystem.feedingCommand(INTAKE_FEEDING_SPEED).
            alongWith(intakePositionSubsystem.moveToAngle(GROUND_INTAKE_ANGLE));
    }

    public Command ampIntake(){
        return intakeFeederSubsystem.feedingCommand(0).
            alongWith(intakePositionSubsystem.moveToAngle(AMP_INTAKE_ANGLE));
    }

    public Command speakerIntake(){
        return intakeFeederSubsystem.feedingCommand(0).
            alongWith(intakePositionSubsystem.moveToAngle(SPEAKER_INTAKE_ANGLE));
    }
    
    public Command shootIntake(double speed){
        return intakeFeederSubsystem.feedingCommand(speed).
            andThen(new ScheduleCommand(Commands.waitSeconds(0.5))).
            andThen(speakerIntake());
    }


}
