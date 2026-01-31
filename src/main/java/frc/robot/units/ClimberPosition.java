package frc.robot.units;


public class ClimberPosition {
    private double motorRots;
    private ClimberPosition (double _motorRotation){
        motorRots = _motorRotation;
    }
    public static ClimberPosition fromMotorRotations (double rotations) {
        return new ClimberPosition(rotations);

    }
    public static ClimberPosition fromMechanismInches (double inches) {
        //return new ClimberPosition(inches);
    }
}
