// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final CommandXboxController driver1;
    private final CommandXboxController driver2;
    private final XboxController hid1, hid2;

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final LimelightSub reeflimelight = new LimelightSub("reef");
    private final LimelightSub funnellimelight = new LimelightSub("funnel");

    private SendableChooser<Command> autoChooser;
    private TeleopDrive teleopDrive;

    public RobotContainer() {
        driver1 = new CommandXboxController(0);
        driver2 = new CommandXboxController(1);
        hid1 = driver1.getHID(); // use hid objects to reduce performance impact. Using getBoolean() on the trigger from
        // CommandXboxController causes large CPU usage
        hid2 = driver2.getHID();

        teleopDrive = new TeleopDrive(
                swerve,
                () -> -MathUtil.applyDeadband(hid1.getLeftY(), SwerveConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(hid1.getLeftX(), SwerveConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(hid1.getRightX(), SwerveConstants.RIGHT_X_DEADBAND),
                () -> hid1.getPOV(),
                () -> hid1.getLeftTriggerAxis() > SwerveConstants.TRIGGER_DEADBAND,
                () -> hid1.getBackButton(),
                () -> MathUtil.applyDeadband(hid1.getRightTriggerAxis(), SwerveConstants.TRIGGER_DEADBAND));
        swerve.setDefaultCommand(teleopDrive);

        configureBindings();

        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    private void configureBindings() {
        driver1.b().onTrue(Commands.runOnce(swerve::zeroGyroWithAlliance));
        driver1.x().whileTrue(Commands.runOnce(swerve::lock, swerve).repeatedly());
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerve;
    }

    public void setTeleopDefaultCommand() {
        swerve.setDefaultCommand(teleopDrive);
    }
}
