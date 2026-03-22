package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.Idle;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimbSubsystem;
import java.util.Optional;

public class AutoDriveToTower extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final ClimbSubsystem climb;
    private final Optional<Time> expiration;

    private Optional<Timer> expirationTimer;

    public AutoDriveToTower(
            CommandSwerveDrivetrain swerveSub, ClimbSubsystem noDepClimbSub, Optional<Time> expirationTime) {
        swerve = swerveSub;
        climb = noDepClimbSub;
        expiration = expirationTime;

        addRequirements(swerve);

        expirationTimer = Optional.empty();
    }

    @Override
    public void initialize() {
        if (expiration.isPresent()) {
            expirationTimer = Optional.of(new Timer());
            expirationTimer.get().start();
        }

        ApplyRobotSpeeds request = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSpeeds(ChassisSpeeds.discretize(
                        new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.of(-0.5), RadiansPerSecond.zero()),
                        0.02));
        Command driveCommand = swerve.applyRequest(() -> request);

        CommandScheduler.getInstance().schedule(driveCommand);
    }

    @Override
    public void end(boolean interrupted) {
        Idle request = new Idle();
        Command idleCommand = swerve.applyRequest(() -> request);

        CommandScheduler.getInstance().schedule(idleCommand);
    }

    @Override
    public boolean isFinished() {
        return expirationTimer.map((val) -> val.hasElapsed(expiration.get())).orElse(false) || climb.canSeeTower();
    }
}
