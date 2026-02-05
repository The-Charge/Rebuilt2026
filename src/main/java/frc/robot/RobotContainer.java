// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.leds.AllianceZoneLED;
import frc.robot.commands.leds.BlinkLED;
import frc.robot.commands.leds.IdleLED;
import frc.robot.commands.leds.NeutralZoneLED;
import frc.robot.commands.leds.RainbowLED;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {

    // singleton instance
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    public final PowerDistribution pdp;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;
    public final LEDSubsystem ledSub;

    private RobotContainer() {
        pdp = new PowerDistribution();

        commandDriver1 = new CommandXboxController(0);
        hidDriver1 = commandDriver1.getHID();
        commandDriver2 = new CommandXboxController(1);
        hidDriver2 = commandDriver2.getHID();

        ledSub = new LEDSubsystem();
        ledSub.setDefaultCommand(new IdleLED(ledSub));

        configureBindings();
    }

    private void configureBindings() {
        commandDriver1.b().onTrue(new BlinkLED(ledSub, Color.kRed));
        commandDriver1.x().onTrue(new RainbowLED(ledSub));
        commandDriver1.y().onTrue(new AllianceZoneLED(ledSub));
        commandDriver1.leftBumper().onTrue(new NeutralZoneLED(ledSub));
        commandDriver1.rightBumper().onTrue(new BlinkLED(ledSub, LEDConstants.orange, Seconds.of(2)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
