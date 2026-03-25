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
import frc.robot.utils.Logger;
import java.util.Optional;

public class AutoDriveToTower extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final ClimbSubsystem climb;
    private final Optional<Time> expiration;

    private Optional<Timer> expirationTimer;
    private boolean hasSeenTower;
    private Optional<Timer> finalAlignTimer;

    public AutoDriveToTower(
            CommandSwerveDrivetrain passSwerveSub, ClimbSubsystem noDepClimbSub, Optional<Time> expirationTime) {
        swerve = passSwerveSub;
        climb = noDepClimbSub;
        expiration = expirationTime;

        expirationTimer = Optional.empty();
        hasSeenTower = false;
        finalAlignTimer = Optional.empty();
    }

    @Override
    public void initialize() {
        expirationTimer = Optional.empty();
        hasSeenTower = false;
        finalAlignTimer = Optional.empty();

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
    public void execute() {
        Logger.println("tsesfsdfsfsefsefsefsefsef");
        if (climb.canSeeTower() && !hasSeenTower) {
            hasSeenTower = true;

            ApplyRobotSpeeds request = new ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSpeeds(ChassisSpeeds.discretize(
                            new ChassisSpeeds(
                                    MetersPerSecond.of(-0.5), MetersPerSecond.zero(), RadiansPerSecond.zero()),
                            0.02));
            Command driveCommand = swerve.applyRequest(() -> request);

            CommandScheduler.getInstance().schedule(driveCommand);

            finalAlignTimer = Optional.of(new Timer());
            finalAlignTimer.get().start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Idle request = new Idle();
        Command idleCommand = swerve.applyRequest(() -> request);

        CommandScheduler.getInstance().schedule(idleCommand);
    }

    @Override
    public boolean isFinished() {
        return false;
        // return expirationTimer.map((val) -> val.hasElapsed(expiration.get())).orElse(false)
        //         || finalAlignTimer.map((val) -> val.hasElapsed(0.25)).orElse(false);
    }
}
