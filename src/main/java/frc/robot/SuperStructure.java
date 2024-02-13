package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;

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
        return intakeFeederSubsystem.feedingCommand(5).
        alongWith(intakePositionSubsystem.moveToAngle(0));
    }

    public Command ampIntake(){
        return intakeFeederSubsystem.feedingCommand(0).
        alongWith(intakePositionSubsystem.moveToAngle(90));
    }

    public Command speakerIntake(){
        return intakeFeederSubsystem.feedingCommand(0).
        alongWith(intakePositionSubsystem.moveToAngle(120));
    }

}
