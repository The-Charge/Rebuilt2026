// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.subsystems.ClimbSubsystem;

public class RobotContainer {

    // singleton instance
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    public final PowerDistribution pdp;
    public final ClimbSubsystem climbe;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;

    private RobotContainer() {
        pdp = new PowerDistribution();

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);
        hidDriver2 = commandDriver2.getHID();

        climbe = new ClimbSubsystem();

        configureBindings();
    }

    private void configureBindings() {
        commandDriver1.a().onTrue(new ClimbUp(climbe));
        commandDriver1.b().onTrue(new ClimbDown(climbe));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
