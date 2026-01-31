// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.vision.AlignTurret;
import frc.robot.commands.vision.LimelightCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.io.File;

public class RobotContainer {
    // singleton instance
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final LimelightSubsystem reeflimelight = new LimelightSubsystem("reef", new Pose3d());
    private final LimelightSubsystem funnellimelight;
    private TeleopDrive teleopDrive;
    public TurretSubsystem turret;

    public final PowerDistribution pdp;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;

    private RobotContainer() {
        pdp = new PowerDistribution();

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);
        hidDriver2 = commandDriver2.getHID();

        funnellimelight = new LimelightSubsystem("funnel", new Pose3d());

        turret = new TurretSubsystem();
        // teleopDrive = new TeleopDrive(
        //         swerve,
        //         () -> -MathUtil.applyDeadband(hidDriver1.getLeftY(), SwerveConstants.LEFT_Y_DEADBAND),
        //         () -> -MathUtil.applyDeadband(hidDriver1.getLeftX(), SwerveConstants.LEFT_X_DEADBAND),
        //         () -> -MathUtil.applyDeadband(hidDriver1.getRightX(), SwerveConstants.RIGHT_X_DEADBAND),
        //         () -> hidDriver1.getPOV(),
        //         () -> hidDriver1.getLeftTriggerAxis() > SwerveConstants.TRIGGER_DEADBAND,
        //         () -> hidDriver1.getBackButton(),
        //         () -> MathUtil.applyDeadband(hidDriver1.getRightTriggerAxis(), SwerveConstants.TRIGGER_DEADBAND));
        // swerve.setDefaultCommand(teleopDrive);

        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
        configureBindings();
    }

    private void configureBindings() {
        commandDriver1.x().onTrue(new AlignTurret(turret, swerve));
        commandDriver1.x().onTrue(new LimelightCommand(funnellimelight, swerve));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
