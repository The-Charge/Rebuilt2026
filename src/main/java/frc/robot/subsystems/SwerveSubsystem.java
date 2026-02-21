package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    public double getSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSpeed'");
    }

    public double getAngularVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAngularVelocity'");
    }

    public Rotation2d getHeading() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getHeading'");
    }

    public Pose2d getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    public void addvisionmeasuremant(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addvisionmeasuremant'");
    }

    public void addVisionReading(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addVisionReading'");
    }

    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }
}
