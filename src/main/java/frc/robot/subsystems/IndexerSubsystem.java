package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX spindexerMotor;
    
    public IndexerSubsystem() {
        
        spindexerMotor = new TalonFX(IndexConstants.spindexerMotorID);//port number under indexConstants

        configureTalonFXMotor(); //always configure before their use
    }
@Override
    public void periodic(){

    }

    public void stop(){
        spindexerMotor.stopMotor();//safety
    }

    private void configureTalonFXMotor(){

    }
}

