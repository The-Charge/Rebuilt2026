package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    private final TalonFX climber;


    public ClimbSubsystem (){
        climber = new TalonFX(0);
        configureTalonFXMotor();


    }

        @Override
    public void periodic() {

    }

     // example code to set the closed-loop (PID) target velocity to a given value
    // this function is public so that it can be called from commands using this subsystem
    public void setClimbMotorPosition(double Position) {
        PositionVoltage  request = new PositionVoltage(Position);
        climber.setControl(request);
    }

    // function to immediately stop all physical movement in the subsystem
    // this is used mostly for safety reasons, such as to stop all movement on the robot when it is disabled
    // this would usually be called in Robot.disabledInit
    public void stop() {
        climber.stopMotor();
      
    }

    // example code to get the current velocity of a motor
    public double getPosition() {
        return climber.getPosition().getValue().abs(Units.Radian);
    }

    // this is a function for use only within the subsystem itself, so it is marked private
    // this is a function to configure/setup a motor when the robot turns on
    private void configureTalonFXMotor() {
        // configuring motors is complex and varies a lot depending on what type of motor you are programming and what
        //     it is being programmed for
        // thus, I have left this function blank. Just ask somebody experienced for help once you get here
    }


    
}
