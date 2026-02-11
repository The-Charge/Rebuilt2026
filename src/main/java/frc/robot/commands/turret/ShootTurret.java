package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPos;
import java.util.Optional;

public class ShootTurret extends Command {

    public ShooterSubsystem.HoodPos hoodPos;

    private final ShooterSubsystem shooterSub;
    private final LimelightSubsystem vSub;

    public ShootTurret(ShooterSubsystem shootSub, LimelightSubsystem vSub) {
        this.shooterSub = shootSub;
        this.vSub = vSub;

        addRequirements(shooterSub);
        addRequirements(vSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Optional<Pose3d> tagDist = vSub.getTransformToTag(20);
        if (tagDist.isEmpty()) {
            return;
        }

        double distance = tagDist.get().getTranslation().getNorm();

        if (distance > ShooterConstants.hoodPosThreshold) { // change the way we get distance ?
            shooterSub.setHoodPos(HoodPos.DOWN); // if close go high
        } else {
            shooterSub.setHoodPos(HoodPos.UP); // if far go far
        }

        shooterSub.shoot(RPM.of(10)); // TODO: turn distance into angluar velocity
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
