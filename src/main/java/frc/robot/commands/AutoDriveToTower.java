package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoDriveToTower extends Command {

    private final CommandSwerveDrivetrain swerve;
    private final ClimbSubsystem climb;

    public AutoDriveToTower(CommandSwerveDrivetrain swerveSub, ClimbSubsystem noDepClimbSub) {
        swerve = swerveSub;
        climb = noDepClimbSub;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        ApplyFieldSpeeds request = new ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);
        // Command driveCommand = swerve.applyRequest(() -> request.with) //WIP
    }
}
