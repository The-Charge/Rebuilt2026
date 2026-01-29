// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private final CommandXboxController driver1;
    private final CommandXboxController driver2;

    public IntakeSubsystem intakeSystem;

    public RobotContainer() {
        intakeSystem = new IntakeSubsystem();

        driver1 = new CommandXboxController(0);
        driver2 = new CommandXboxController(1);

        configureBindings();
    }

    private void configureBindings() {
        driver2.a().onTrue(new DeployIntake(intakeSystem));
        driver2.b().onTrue(new RetractIntake(intakeSystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
