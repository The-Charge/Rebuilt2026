// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.teleop.TeleopLogic;
import frc.robot.utils.Alerts;
import frc.robot.utils.AutoDisplayUtil;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.ControllerUtil;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import java.util.Optional;

public class Robot extends TimedRobot {

    private static Robot instance;

    private Command autoCommand;
    private Optional<TeleopLogic> teleopLogic;
    private Optional<TeleopDrive> teleopDrive;
    private Optional<Timer> autoGyroTimer;

    private SlewRateLimiter pdpVoltageSlew;

    public Robot() {
        instance = this;

        Logger.init(this); // DO NOT DELETE ; start logger
        RobotContainer.getInstance(); // DO NOT DELETE ; create singleton instance
        Alerts.setupDeferredInitializations();

        // handle disconnect of CAN devices;
        // set callback function to log reconnect and flash LEDs for disconnection
        CANMonitor.setConnectionChangeCallback((id, connected) -> {
            if (connected == true) {
                Logger.println(String.format("Connected to CAN device %d", id));
                return;
            }

            Logger.reportWarning(String.format("Lost connection to CAN device %d", id), false);
            CommandScheduler.getInstance()
                    .schedule(new BlinkLED(RobotContainer.getInstance().ledSub, Color.kRed, Seconds.of(2))
                            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        });

        teleopLogic = Optional.empty();
        teleopDrive = Optional.empty();
        autoGyroTimer = Optional.empty();

        /*
         * Allow an increase at a rate of 100 volts / second (effectively infinite)
         * Allow a derease at a rate of 0.2 volts / second
         * Start with the current pdp reading
         */
        pdpVoltageSlew =
                new SlewRateLimiter(100, 0.2, RobotContainer.getInstance().pdp.getVoltage());

        addPeriodic(this::slowRobotPeriodic, Seconds.of(0.05));
        addPeriodic(this::verySlowRobotPeriodic, Seconds.of(0.5));
    }

    public static Robot getInstance() {
        return instance;
    }

    public Optional<TeleopLogic> getTeleopLogic() {
        return teleopLogic;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.logDouble("", "matchTime", DriverStation.getMatchTime());
        Logger.logBool("", "isReadyToShoot", RobotContainer.getInstance().isReadyToShoot());
    }

    public void slowRobotPeriodic() {
        RobotContainer.getInstance().climber.slowPeriodic();
        RobotContainer.getInstance().indexer.slowPeriodic();
        RobotContainer.getInstance().intake.slowPeriodic();
        RobotContainer.getInstance().ledSub.slowPeriodic();
        RobotContainer.getInstance().limelights.slowPeriodic();
        RobotContainer.getInstance().auxSwerve.slowPeriodic();
        RobotContainer.getInstance().turret.slowPeriodic();
        RobotContainer.getInstance().shooter.slowPeriodic();

        Logger.logPDP(RobotContainer.getInstance().pdp);

        ControllerUtil.periodic(RobotContainer.getInstance().hidDriver1, RobotContainer.getInstance().hidDriver2);
    }

    public void verySlowRobotPeriodic() {
        RobotContainer.getInstance().climber.verySlowPeriodic();
        RobotContainer.getInstance().indexer.verySlowPeriodic();
        RobotContainer.getInstance().intake.verySlowPeriodic();
        RobotContainer.getInstance().ledSub.verySlowPeriodic();
        RobotContainer.getInstance().limelights.verySlowPeriodic();
        RobotContainer.getInstance().auxSwerve.verySlowPeriodic();
        RobotContainer.getInstance().turret.verySlowPeriodic();
        RobotContainer.getInstance().shooter.verySlowPeriodic();

        boolean pdpConnected = MiscUtils.isPDPConnected(RobotContainer.getInstance().pdp);
        CANMonitor.logCANDeviceStatus("PDP", RobotContainer.getInstance().pdp.getModule() + 1, pdpConnected);
        if (Alerts.pdpDisconnected.isPresent()) {
            Alerts.pdpDisconnected.get().set(!pdpConnected);
        }

        double slewedPDPVoltage =
                pdpVoltageSlew.calculate(RobotContainer.getInstance().pdp.getVoltage());
        if (slewedPDPVoltage <= 11) {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(true);
        } else if (slewedPDPVoltage <= 12) {
            Alerts.lowBattery.set(true);
            Alerts.criticalBattery.set(false);
        } else {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(false);
        }

        Alerts.driver1Missing.set(!RobotContainer.getInstance().hidDriver1.isConnected());
        Alerts.driver2Missing.set(!RobotContainer.getInstance().hidDriver2.isConnected());
        Alerts.buttonBoxConnected.set(RobotContainer.getInstance().hidButtonBox.isConnected());
        Alerts.fmsConnected.set(DriverStation.isFMSAttached());
    }

    @Override
    public void disabledInit() {
        RobotContainer.getInstance().indexer.stopAll();
        RobotContainer.getInstance().intake.stopRoller();
        RobotContainer.getInstance().climber.stopAll();
        RobotContainer.getInstance().shooter.stopShooter();
        RobotContainer.getInstance().turret.stopTurret();

        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().idleLEDCommand, true);

        ControllerUtil.cancelControllerRumbles(0);
        ControllerUtil.cancelControllerRumbles(1);

        if (DriverStation.isFMSAttached()) {
            RobotContainer.getInstance().limelights.takeRewind();
        }

        RobotContainer.getInstance().displayAuto();

        RobotContainer.getInstance().limelights.setThrottleMode(true);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().autoLEDCommand, true);

        autoCommand = RobotContainer.getInstance().getAutonomousCommand();
        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }

        RobotContainer.getInstance().displayAuto();

        autoGyroTimer = Optional.of(new Timer());
        autoGyroTimer.get().start();
        // RobotContainer.getInstance().limelightCommand.cancel();
        onEnabled();
    }

    @Override
    public void autonomousPeriodic() {
        if (autoGyroTimer.isPresent() && autoGyroTimer.get().hasElapsed(0.1)) {
            // CommandScheduler.getInstance().schedule(RobotContainer.getInstance().limelightCommand);
            RobotContainer.getInstance().limelights.seedSwerve();
            autoGyroTimer = Optional.empty();
        }
    }

    @Override
    public void autonomousExit() {
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        try {
            AutoDisplayUtil.clearAutoPath();
        } catch (Exception e) {
            Logger.reportError(e);
        }

        teleopLogic = Optional.of(new TeleopLogic());
        teleopDrive = Optional.of(new TeleopDrive());

        onEnabled();
    }

    @Override
    public void teleopPeriodic() {
        if (teleopLogic.isPresent()) {
            teleopLogic.get().teleopPeriodic();
        }
        if (teleopDrive.isPresent()) {
            teleopDrive.get().teleopPeriodic();
        }
    }

    @Override
    public void teleopExit() {
        if (teleopLogic.isPresent()) {
            teleopLogic.get().endTeleop();
        }
        if (teleopDrive.isPresent()) {
            teleopDrive.get().endTeleop();
        }
        teleopLogic = Optional.empty();
        teleopDrive = Optional.empty();
    }

    @Override
    public void testInit() {
        RobotContainer.getInstance().turret.removeDefaultCommand();
        RobotContainer.getInstance().shooter.removeDefaultCommand();
        RobotContainer.getInstance().intake.removeDefaultCommand();
        // RobotContainer.getInstance().indexer.removeDefaultCommand();
        RobotContainer.getInstance().climber.removeDefaultCommand();

        teleopDrive = Optional.of(new TeleopDrive());
        onEnabled();
    }

    @Override
    public void testPeriodic() {
        if (teleopDrive.isPresent()) {
            teleopDrive.get().teleopPeriodic();
        }
    }

    @Override
    public void testExit() {
        if (teleopDrive.isPresent()) {
            teleopDrive.get().endTeleop();
        }
        teleopDrive = Optional.empty();
    }

    private void onEnabled() {
        RobotContainer.getInstance().limelights.setThrottleMode(false);
    }
}
