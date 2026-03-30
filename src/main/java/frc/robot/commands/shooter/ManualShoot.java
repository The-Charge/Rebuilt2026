package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class ManualShoot extends Command {

    private final ShooterSubsystem shoot;
    private final Supplier<AngularVelocity> targetVel;
    private final boolean override;

    private final String key = String.format("%s/speed", getName());

    public ManualShoot(ShooterSubsystem shootSub, Supplier<AngularVelocity> vel, boolean allowOverride) {
        shoot = shootSub;
        targetVel = vel;
        override = allowOverride;

        addRequirements(shoot);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putNumber(key, 0);
        }
    }

    @Override
    public void execute() {
        if (ShooterConstants.manualShootUseSmartdashboard && override) {
            shoot.setTargetVelocity(RPM.of(SmartDashboard.getNumber(key, 0)));
        } else {
            shoot.setTargetVelocity(targetVel.get());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
