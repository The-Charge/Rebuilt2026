// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.FieldZone;
import frc.robot.utils.Logger;
import java.util.Optional;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private Optional<Pose2d> lastRobotPose;

    public Robot() {
        Logger.init(); // DO NOT DELETE ; start logger
        RobotContainer.getInstance(); // DO NOT DELETE ; create singleton instance

        CANMonitor.setConnectionChangeCallback((id, connected) -> {
            if (connected == true) {
                Logger.println(String.format("Reconnected to CAN device %d", id));
                return;
            }

            Logger.reportWarning(String.format("Lost connection to CAN device %d", id), false);
            CommandScheduler.getInstance()
                    .schedule(new BlinkLED(RobotContainer.getInstance().ledSub, Color.kRed, Seconds.of(2)));
        });

        lastRobotPose = Optional.empty();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.logPDP(RobotContainer.getInstance().pdp);
        CANMonitor.logCANDeviceStatus(
                "PDP",
                RobotContainer.getInstance().pdp.getModule(),
                CANMonitor.isPDPConnected(RobotContainer.getInstance().pdp));
        Alerts.pdpDisconnected.set(!CANMonitor.isPDPConnected(RobotContainer.getInstance().pdp));

        Alerts.driver1Missing.set(!RobotContainer.getInstance().hidDriver1.isConnected());
        Alerts.driver2Missing.set(!RobotContainer.getInstance().hidDriver2.isConnected());
        Alerts.fmsConnected.set(DriverStation.isFMSAttached());

        double batteryVoltage = RobotContainer.getInstance().pdp.getVoltage();
        if (batteryVoltage <= 10) {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(true);
        } else if (batteryVoltage <= 11) {
            Alerts.lowBattery.set(true);
            Alerts.criticalBattery.set(false);
        } else {
            Alerts.lowBattery.set(false);
            Alerts.criticalBattery.set(false);
        }
    }

    @Override
    public void disabledInit() {
        RobotContainer.getInstance().indexer.stopAll();
        RobotContainer.getInstance().intake.stopRoller();
        RobotContainer.getInstance().climber.stopAll();

        RobotContainer.getInstance().ledSub.setDefaultCommand(RobotContainer.getInstance().idleLEDCommand);
        lastRobotPose = Optional.empty();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    public void enabledPeriodic() {
        Pose2d robotPose = Pose2d.kZero; // TODO: replace with value from odo

        FieldZone currentZone = FieldZone.fromRobotPose(robotPose);
        Optional<FieldZone> lastZone = lastRobotPose.map((val) -> FieldZone.fromRobotPose(val));
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (lastZone.isEmpty() || lastZone.get() != currentZone) {
            // spotless:off
            if (
                currentZone == FieldZone.BLUE && alliance == Alliance.Blue
                ||
                currentZone == FieldZone.RED && alliance == Alliance.Red
            ) {
            // spotless:on
                RobotContainer.getInstance()
                        .ledSub
                        .setDefaultCommand(RobotContainer.getInstance().friendlyZoneLEDCommand);

                CommandScheduler.getInstance().schedule(RobotContainer.getInstance().spinUpIndexerCommand);
                // TODO: aim turret at hub
            } else if (currentZone == FieldZone.NEUTRAL) {
                RobotContainer.getInstance()
                        .ledSub
                        .setDefaultCommand(RobotContainer.getInstance().neutralZoneLEDCommand);

                CommandScheduler.getInstance().schedule(RobotContainer.getInstance().spinDownIndexerCommand);
                // TODO: aim turret at nearest gap to friendly alliance zone
            } else {
                RobotContainer.getInstance()
                        .ledSub
                        .setDefaultCommand(RobotContainer.getInstance().opposingZoneLEDCommand);

                CommandScheduler.getInstance().schedule(RobotContainer.getInstance().spinDownIndexerCommand);
                // TODO: recenter turret
            }
        }

        lastRobotPose = Optional.of(robotPose);
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = RobotContainer.getInstance().getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        enabledPeriodic();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().schedule(new DeployIntake(RobotContainer.getInstance().intake));
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
