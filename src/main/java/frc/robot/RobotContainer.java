// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.vision.AlignTurret;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    // singleton instance
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    public final TurretSubsystem turret;
    public final LimelightSubsystem turretLimelight;
    public final SwerveSubsystem swerveSubsystem;
    public final PowerDistribution pdp;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;

    private RobotContainer() {
        pdp = new PowerDistribution();

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);
        hidDriver2 = commandDriver2.getHID();

        turret = new TurretSubsystem();
        turretLimelight = new LimelightSubsystem("turret");
        swerveSubsystem = new SwerveSubsystem();

        configureBindings();
    }

    private void configureBindings() {
        commandDriver1.x().onTrue(new AlignTurret(turretLimelight, turret, swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
