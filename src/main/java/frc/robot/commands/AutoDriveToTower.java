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
    private Command towardCommand, finalAlignCommand;

    public AutoDriveToTower(
            CommandSwerveDrivetrain passSwerveSub, ClimbSubsystem noDepClimbSub, Optional<Time> expirationTime) {
        swerve = passSwerveSub;
        climb = noDepClimbSub;
        expiration = expirationTime;
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        hasSeenTower = false;
        finalAlignTimer = Optional.empty();

        if (expiration.isPresent()) {
            expirationTimer = Optional.of(new Timer());
            expirationTimer.get().start();
        } else {
            expirationTimer = Optional.empty();
        }

        ApplyRobotSpeeds towardRequest = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSpeeds(ChassisSpeeds.discretize(
                        new ChassisSpeeds(MetersPerSecond.zero(), MetersPerSecond.of(-0.5), RadiansPerSecond.zero()),
                        0.02));
        towardCommand = swerve.applyRequest(() -> towardRequest);

        ApplyRobotSpeeds finalAlignRequest = new ApplyRobotSpeeds()
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSpeeds(ChassisSpeeds.discretize(
                        new ChassisSpeeds(MetersPerSecond.of(-0.5), MetersPerSecond.zero(), RadiansPerSecond.zero()),
                        0.02));
        finalAlignCommand = swerve.applyRequest(() -> finalAlignRequest);

        towardCommand.initialize();
    }

    @Override
    public void execute() {
        Logger.println("tsesfsdfsfsefsefsefsefsef");

        Logger.logBool(getName(), "hasSeenTower", hasSeenTower);

        if (!hasSeenTower) {
            towardCommand.execute();
        } else {
            finalAlignCommand.execute();
        }

        if (climb.canSeeTower() && !hasSeenTower) {
            hasSeenTower = true;

            finalAlignTimer = Optional.of(new Timer());
            finalAlignTimer.get().start();

            towardCommand.end(true);
            finalAlignCommand.initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        finalAlignCommand.end(true);

        Idle request = new Idle();
        Command idleCommand = swerve.applyRequest(() -> request);

        idleCommand.initialize();
        idleCommand.execute();
        idleCommand.end(true);
    }

    @Override
    public boolean isFinished() {
        return expirationTimer.map((val) -> val.hasElapsed(expiration.get())).orElse(false)
                || finalAlignTimer.map((val) -> val.hasElapsed(0.25)).orElse(false);
    }
}
