package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPos;

public class ShootTurret extends Command {
    public ShooterSubsystem shootSub;

    public ShooterSubsystem.HoodPos hoodPos;
    
        private LimelightSubsystem vSub;
    
        public ShootTurret(ShooterSubsystem shootSub, LimelightSubsystem vSub) {
            this.shootSub = shootSub;
            this.vSub = vSub;

        addRequirements(shootSub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Optional<Pose3d> tagDist = vSub.getTransformToTag(20);
        if (tagDist.isEmpty()) {
            return;
        }
        if (tagDist.get().getTranslation().getNorm() > ShooterConstants.hoodPosThreshold) { // change the way we get distance ?
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
