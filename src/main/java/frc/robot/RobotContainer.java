// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

    private void setupSwerve() {
        swerveFieldCentricDrive = new SwerveRequest.FieldCentric()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerveRobotCentricDrive = new SwerveRequest.RobotCentric()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        swerveFieldCentricFacingAngleDrive = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(MetersPerSecond.of(0.01))
                .withRotationalDeadband(RotationsPerSecond.of(0.01))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        swerveBrake = new SwerveRequest.SwerveDriveBrake();
        swerveIdle = new SwerveRequest.Idle();

        DoubleSupplier speedShifter = () -> hidDriver1.getRightTriggerAxis() >= 0.5 ? 0.25 : 1;

        swerveFieldCentricDriveCommand = swerve.applyRequest(() -> swerveFieldCentricDrive
                .withVelocityX(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftY(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()))
                .withVelocityY(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftX(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()))
                .withRotationalRate(SwerveConstants.maxAngularVel.times(
                        ControllerUtil.applyLinearDeadband(-hidDriver1.getRightX(), SwerveConstants.joystickDeadband)
                                * speedShifter.getAsDouble())));

        swerveRobotCentricDriveCommand = swerve.applyRequest(() -> swerveRobotCentricDrive
                .withVelocityX(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftY(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()))
                .withVelocityY(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                -hidDriver1.getLeftX(),
                                SwerveConstants.joystickDeadband,
                                SwerveConstants.joystickExponent)
                        * speedShifter.getAsDouble()))
                .withRotationalRate(SwerveConstants.maxAngularVel.times(
                        ControllerUtil.applyLinearDeadband(-hidDriver1.getRightX(), SwerveConstants.joystickDeadband)
                                * speedShifter.getAsDouble())));

        swerveFieldCentricFacingAngleDriveCommand = swerve.applyRequest(() -> {
            ChassisSpeeds speed = swerve.getStateCopy().Speeds;
            if (Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) > 0.01) {
                lastDefinedRotation =
                        Optional.of(new Rotation2d(Math.atan2(speed.vyMetersPerSecond, speed.vxMetersPerSecond)));
            }

            var req = swerveFieldCentricFacingAngleDrive
                    .withVelocityX(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                    -hidDriver1.getLeftY(),
                                    SwerveConstants.joystickDeadband,
                                    SwerveConstants.joystickExponent)
                            * speedShifter.getAsDouble()))
                    .withVelocityY(SwerveConstants.maxTranslationVel.times(ControllerUtil.applyExponentialDeadband(
                                    -hidDriver1.getLeftX(),
                                    SwerveConstants.joystickDeadband,
                                    SwerveConstants.joystickExponent)
                            * speedShifter.getAsDouble()));
            if (lastDefinedRotation.isPresent()) {
                req = req.withTargetDirection(lastDefinedRotation.get());
            }

            return req;
        });

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(swerveFieldCentricDriveCommand);
        commandDriver1
                .b()
                .onTrue(new InstantCommand(() -> {
                            swerve.resetRotation(
                                    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                            ? Rotation2d.kZero
                                            : Rotation2d.k180deg);
                        })
                        .ignoringDisable(true));
        commandDriver1.x().whileTrue(swerve.applyRequest(() -> swerveBrake));
        commandDriver1
                .leftTrigger()
                .onTrue(new InstantCommand(() ->
                                MiscUtils.changeSubsystemDefaultCommand(swerve, swerveRobotCentricDriveCommand, false))
                        .ignoringDisable(true))
                .onFalse(new InstantCommand(() ->
                                MiscUtils.changeSubsystemDefaultCommand(swerve, swerveFieldCentricDriveCommand, false))
                        .ignoringDisable(true));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled()
                .whileTrue(swerve.applyRequest(() -> swerveIdle).ignoringDisable(true))
                .onTrue(new InstantCommand(() -> {
                            lastDefinedRotation = Optional.empty();
                        })
                        .ignoringDisable(true)); // reset snake mode
    }

    private void configureBindings() {
        // TODO: make swerve turn so that intake automatically faces the direction of travel while the intake is running
        commandDriver2
                .leftTrigger()
                .whileTrue(new ParallelCommandGroup(
                        new RunRoller(intake), new ScheduleCommand(new BlinkLED(ledSub, Color.kWhite))));
        commandDriver2.povUp().onTrue(new ClimbUp(climber, false));
        commandDriver2.povDown().onTrue(new ClimbDown(climber, false));
        commandDriver2
                .rightTrigger()
                .whileTrue(new RepeatCommand(new ConditionalCommand(
                        new SpinUpIndexer(indexer), new SpinDownIndexer(indexer), () -> isReadyToShoot())))
                .onFalse(new SpinDownIndexer(indexer));

        commandButtonBox
                .resetTurret()
                .onTrue(new CalibrateTurret(turret).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox.zeroClimb().onTrue(new InstantCommand(() -> climber.setAsZero()));
        commandButtonBox
                .deployIntake()
                .onTrue(new DeployIntake(intake).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        // commandButtonBox.disableOdo().onTrue();
        commandButtonBox
                .turretLeft()
                .whileTrue(new ManualTurret(turret, TurretConstants.manualSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .turretRight()
                .whileTrue(new ManualTurret(turret, -TurretConstants.manualSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .testShoot()
                .onTrue(new ManualShoot(
                                shooter,
                                () -> ShooterConstants.ShootConfig.maxManualSpeed.times(
                                        (-hidButtonBox.getSliderAxis() + 1) / 2.0))
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        commandButtonBox
                .stopShoot()
                .onTrue(new StopShooter(shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox
                .spoolDown()
                .whileTrue(new ManualSpool(climber, ClimberConstants.manualSpoolSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        commandButtonBox
                .spoolUp()
                .whileTrue(new ManualSpool(climber, -ClimberConstants.manualSpoolSpeed)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    }

    private void addNamedCommands() {
        // NamedCommands.registerCommand("ClimbUp", new ClimbUp(climber, true));
        // NamedCommands.registerCommand("ClimbDown", new ClimbDown(climber, true));
        // NamedCommands.registerCommand(
        //         "PrepShootAtHub", new ParallelCommandGroup(prepShootAtHubCommand, pointAtHubCommand));
        // NamedCommands.registerCommand("Shoot", new SpinUpIndexer(indexer, false));
        // NamedCommands.registerCommand(
        //         "StopShoot", new ParallelCommandGroup(new StopShooter(shooter), new SpinDownIndexer(indexer)));
        // NamedCommands.registerCommand("CalibrateTurret", new CalibrateTurret(turret));
        // NamedCommands.registerCommand("WaitForReadyToShoot", new WaitUntilCommand(() -> isReadyToShoot()));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
