// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopCommand;
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

    private Command m_autonomousCommand;
    private Optional<TeleopLogic> teleopLogic;
    private Optional<TeleopCommand> teleopCommand;
    private Optional<Timer> autoGyroTimer;

    public Robot() {
        instance = this;

        Logger.init(this); // DO NOT DELETE ; start logger
        RobotContainer.getInstance(); // DO NOT DELETE ; create singleton instance

        // handle disconnect of CAN devices;
        // set Callback function to log reconnect and flash LEDs for disconnection
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
        teleopCommand = Optional.empty();
        autoGyroTimer = Optional.empty();

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
        RobotContainer.getInstance().otherLimelight.slowPeriodic();
        RobotContainer.getInstance().turretLimelight.slowPeriodic();
        RobotContainer.getInstance().auxSwerve.slowPeriodic();

        Logger.logPDP(RobotContainer.getInstance().pdp);

        ControllerUtil.periodic(RobotContainer.getInstance().hidDriver1, RobotContainer.getInstance().hidDriver2);
    }

    public void verySlowRobotPeriodic() {
        RobotContainer.getInstance().climber.verySlowPeriodic();
        RobotContainer.getInstance().indexer.verySlowPeriodic();
        RobotContainer.getInstance().intake.verySlowPeriodic();
        RobotContainer.getInstance().ledSub.verySlowPeriodic();
        RobotContainer.getInstance().otherLimelight.verySlowPeriodic();
        RobotContainer.getInstance().turretLimelight.verySlowPeriodic();
        RobotContainer.getInstance().auxSwerve.verySlowPeriodic();

        boolean pdpConnected = MiscUtils.isPDPConnected(RobotContainer.getInstance().pdp);
        CANMonitor.logCANDeviceStatus("PDP", RobotContainer.getInstance().pdp.getModule() + 1, pdpConnected);
        Alerts.pdpDisconnected.set(!pdpConnected);

        double batteryVoltage = RobotContainer.getInstance().pdp.getVoltage();
        if (batteryVoltage <= 11) {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(true);
        } else if (batteryVoltage <= 12) {
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
            RobotContainer.getInstance().turretLimelight.takeRewind();
            RobotContainer.getInstance().otherLimelight.takeRewind();
        }
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        MiscUtils.changeSubsystemDefaultCommand(
                RobotContainer.getInstance().ledSub, RobotContainer.getInstance().autoLEDCommand, true);

        m_autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        RobotContainer.getInstance().displayAuto();

        autoGyroTimer = Optional.of(new Timer());
        autoGyroTimer.get().start();
        RobotContainer.getInstance().limelightCommand.cancel();
    }

    @Override
    public void autonomousPeriodic() {
        if (autoGyroTimer.isPresent() && autoGyroTimer.get().hasElapsed(0.1)) {
            CommandScheduler.getInstance().schedule(RobotContainer.getInstance().limelightCommand);
            // RobotContainer.getInstance().limelightCommand.seedFromIMU();
            autoGyroTimer = Optional.empty();
        }
    }

    @Override
    public void autonomousExit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
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
        teleopCommand = Optional.of(new TeleopCommand());
    }

    @Override
    public void teleopPeriodic() {
        if (teleopLogic.isPresent()) {
            teleopLogic.get().teleopPeriodic();
        }
        if (teleopCommand.isPresent()) {
            teleopCommand.get().teleopPeriodic();
        }
    }

    @Override
    public void teleopExit() {
        if (teleopLogic.isPresent()) {
            teleopLogic.get().endTeleop();
        }
        if (teleopCommand.isPresent()) {
            teleopCommand.get().endTeleop();
        }
        teleopLogic = Optional.empty();
        teleopCommand = Optional.empty();
    }

    @Override
    public void testInit() {
        RobotContainer.getInstance().turret.removeDefaultCommand();
        RobotContainer.getInstance().shooter.removeDefaultCommand();
        RobotContainer.getInstance().intake.removeDefaultCommand();
        RobotContainer.getInstance().indexer.removeDefaultCommand();
        RobotContainer.getInstance().climber.removeDefaultCommand();
    }

    @Override
    public void testPeriodic() {}
}
