package frc.robot.utils;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoDisplayUtil {

    private AutoDisplayUtil() {}

    public static void displayAutoPath(Command autoCommand) {
        try {
            // The name is "InstantCommand" when Command.none() is passed
            if (autoCommand == null || autoCommand.getName().equals("InstantCommand")) {
                clearAutoPath();
                return;
            }

            List<PathPlannerPath> paths;
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoCommand.getName());

            // generate each trajectory and merge them into one
            Optional<Trajectory> mergedTraj = Optional.empty();
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            for (PathPlannerPath path : paths) {
                if (isRed) path = path.flipPath();

                List<Pose2d> poses = getPosesFromPath(path);
                Trajectory traj = generateTrajectory(poses);

                if (mergedTraj.isEmpty()) {
                    mergedTraj = Optional.of(traj);
                    continue;
                }
                mergedTraj = Optional.of(mergedTraj.get().concatenate(traj));
            }

            if (mergedTraj.isEmpty()) {
                Logger.reportWarning("Auto has no paths, redirecting to clearAutoPath", false);
                clearAutoPath();
                return;
            }

            RobotContainer.getInstance().ntField.getObject("traj").setTrajectory(mergedTraj.get());
        } catch (Exception e) {
            Logger.reportWarning(e, true);
            return;
        }
    }

    public static void clearAutoPath() {
        try {
            RobotContainer.getInstance().ntField.getObject("traj").setTrajectory(new Trajectory());
        } catch (Exception e) {
            Logger.reportWarning(e, true);
            return;
        }
    }

    private static Trajectory generateTrajectory(List<Pose2d> poses) {
        TrajectoryConfig trajConfig = new TrajectoryConfig(3, 3); // values don't matter as long as they aren't zero

        return TrajectoryGenerator.generateTrajectory(poses, trajConfig);
    }

    private static Rotation2d calculatePoseRotation(PathPoint current, PathPoint next) {
        // calculate rotation from current point to next point using atan2
        double rad = Math.atan2(
                next.position.getY() - current.position.getY(), next.position.getX() - current.position.getX());
        return new Rotation2d(rad);
    }

    private static List<Pose2d> getPosesFromPath(PathPlannerPath path) {
        List<PathPoint> pathPoints = path.getAllPathPoints();
        List<Pose2d> poses = new ArrayList<>();

        for (int i = 0; i < pathPoints.size(); i++) {
            PathPoint pp = pathPoints.get(i);

            Rotation2d rot;
            if (i == 0) {
                if (pathPoints.size() > 1) {
                    rot = calculatePoseRotation(
                            pp, pathPoints.get(i + 1)); // For the first point, use the heading towards the second point
                } else {
                    rot = Rotation2d.kZero; // If there's only one point, use a default rotation
                }
            } else if (i == pathPoints.size() - 1) {
                rot = calculatePoseRotation(
                        pathPoints.get(i - 1), pp); // For the last point, use the heading from the second-to-last point
            } else {
                // For intermediate points, calculate the heading as the average of the direction to the next and
                // previous points
                Rotation2d toNext = calculatePoseRotation(pp, pathPoints.get(i + 1));
                Rotation2d fromPrev = calculatePoseRotation(pathPoints.get(i - 1), pp);
                rot = new Rotation2d((toNext.getRadians() + fromPrev.getRadians()) / 2.0);
            }

            poses.add(new Pose2d(pp.position, rot));
        }

        return poses;
    }
}
