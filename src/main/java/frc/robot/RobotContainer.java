// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.vision.AlignTurret;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
    public TurretSubsystem turret;
    public LimelightSub turretLimelight;

    public RobotContainer() {
        configureBindings();

        turret = new TurretSubsystem();
        turretLimelight = new LimelightSub("turret");

        turret.setDefaultCommand(new AlignTurret(turretLimelight, turret));
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
