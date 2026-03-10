package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Logger;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// Set shooter velocity for correct speed based on hub distance
public class PrepShootAtHub extends Command {

    // public ShooterSubsystem.HoodPos hoodPos;

    private final ShooterSubsystem shooterSub;
    private final LimelightSubsystem vSub;
    private final CommandSwerveDrivetrain swerveSub;
    private final BooleanSupplier isRed;
    private Optional<Alliance> knownAlliance;

    public PrepShootAtHub(
            ShooterSubsystem shootSub,
            LimelightSubsystem vSub,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Optional<Alliance>> alliance) {
        this.shooterSub = shootSub;
        this.vSub = vSub;
        this.swerveSub = swerveSub;
        this.knownAlliance = Optional.empty();
        this.isRed = () -> {
            knownAlliance = knownAlliance.or(alliance);
            return knownAlliance.orElse(Alliance.Blue) == Alliance.Red;
        };
        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putNumber(getName(), 0)
    }

    @Override
    public void execute() {
        Translation2d offsetToHub;

        // Use HubTag directly for distance
        // Optional<Pose3d> hubTagPose = vSub.getTransformToTag(FieldConstants.getHubTag(isRed.getAsBoolean()));

        // Otherwise use swerve positioning
        // if (hubTagPose.isPresent()) {
        //     offsetToHub = hubTagPose.get().getTranslation().toTranslation2d();
        // } else {
        // Get Swerve Distance to Hub
        offsetToHub = FieldConstants.getHubLoc(isRed.getAsBoolean())
                .minus(RobotContainer.getInstance()
                        .turret
                        .getTurretPoseOnField()
                        .getTranslation());
        // }
        Distance distToTarget = Meters.of(Math.hypot(
                offsetToHub.getMeasureX().in(Meters), offsetToHub.getMeasureY().in(Meters)));

        Logger.logDouble(ShooterConstants.subsystemName, "distToTarget", distToTarget.in(Meters));

        // if (distance > ShooterConstants.hoodPosThreshold) {
        //     // if far, shoot low and far
        //     shooterSub.setHoodPos(HoodPos.DOWN);
        // } else {
        //     // if close, shoot high
        //     shooterSub.setHoodPos(HoodPos.UP);
        // }

        shooterSub.shoot(
                RPM.of(ShooterConstants.distanceToRPMPlot.get(distToTarget.in(Meters)))); // Used to be: RPM.of(10)
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
