// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SuperStructure;

/** Add your docs here. */
public class Autos {
    static SuperStructure structure;

    public Autos(){
        structure = new SuperStructure(null);
    }
    
    public static Command shootingAuto(){
        return structure.warmShooter(1)
            .raceWith(new WaitCommand(5))
            .andThen(structure.shootIntake(INTAKE_SHOOTING_SPEED));

    }
}
