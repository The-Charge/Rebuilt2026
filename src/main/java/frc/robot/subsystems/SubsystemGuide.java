package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ConstantsGuide;

// it is good practice to add "Subsystem" to the end of the name of your subsystem, so that we can easily tell what
//     it is just by looking at the code
//     e.g. don't name a subsystem "Intake", name it "IntakeSubsystem"
// extending SubsystemBase tells Java that this class is a subsystem
public class SubsystemGuide extends SubsystemBase {

    // How to define motors:
    // note that they are defined as private and final
    private final TalonFX exampleTalonFXMotor;
    private final SparkMax exampleSparkMaxMotor;
    private final SparkFlex exampleSparkFlexMotor;

    // this is a constructor, it will run when we initialize this subsystem, which is usually only once when the robot
    //     turns on
    // replace "SubsystemGuide" with the class name of your subsystem
    public SubsystemGuide() {
        // initialize example motors
        // use values from constants file instead of hard-coding it. This makes it much easier to quickly reconfigure
        //     the robot in time-limited situations
        exampleTalonFXMotor = new TalonFX(ConstantsGuide.exampleTalonFXID);
        // we use almost exclusively brushless motors, but if you are unsure, just ask
        exampleSparkMaxMotor = new SparkMax(ConstantsGuide.exampleSparkMaxID, MotorType.kBrushless);
        exampleSparkFlexMotor = new SparkFlex(ConstantsGuide.exampleSparkFlexID, MotorType.kBrushless);

        // configure motors before we use them for anything
        configureTalonFXMotor();
    }

    @Override
    public void periodic() {
        // periodic will run once every 'frame' while the subsystem is active, i.e. when the robot is on

        // this function is used mostly for logging debug data and some other minor stuff
        // if you don't know what should go in here, don't worry about it and just leave it empty
    }

    // example code to set the closed-loop (PID) target velocity to a given value
    // this function is public so that it can be called from commands using this subsystem
    public void setExampleMotorVelocity(double targetVel) {
        VelocityVoltage request = new VelocityVoltage(targetVel);
        exampleTalonFXMotor.setControl(request);
    }

    // function to immediately stop all physical movement in the subsystem
    // this is used mostly for safety reasons, such as to stop all movement on the robot when it is disabled
    // this would usually be called in Robot.disabledInit
    public void stop() {
        exampleTalonFXMotor.stopMotor();
        exampleSparkMaxMotor.stopMotor();
        exampleSparkFlexMotor.stopMotor();
    }

    // example code to get the current velocity of a motor
    public double getExampleMotorRPM() {
        return exampleTalonFXMotor.getVelocity().getValue().abs(Units.RPM);
    }

    // this is a function for use only within the subsystem itself, so it is marked private
    // this is a function to configure/setup a motor when the robot turns on
    private void configureTalonFXMotor() {
        // configuring motors is complex and varies a lot depending on what type of motor you are programming and what
        //     it is being programmed for
        // thus, I have left this function blank. Just ask somebody experienced for help once you get here
    }
}
