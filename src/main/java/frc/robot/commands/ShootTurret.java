package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPos;

public class ShootTurret extends Command {
    public ShooterSubsystem shootSub;

    public ShooterSubsystem.HoodPos hoodPos;
    
        private VisionSubsystem vSub;
    
        public ShootTurret(ShooterSubsystem shootSub, VisionSubsystem vSub) {
            this.shootSub = shootSub;
            this.vSub = vSub;

        addRequirements(shootSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (vSub.getTransformToTag().magnitude > ShooterSubsystem.hoodPosThreshold) { // change the way we get distance ?
            shootSub.setHoodPos(HoodPos.DOWN); // if close go high
        } else {
            shootSub.setHoodPos(HoodPos.UP); // if far go far
        }
        shootSub.shoot();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
