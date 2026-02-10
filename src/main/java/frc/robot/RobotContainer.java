// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    // singleton instance
    private static RobotContainer instance = null;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    public final PowerDistribution pdp;

    public final CommandXboxController commandDriver1, commandDriver2;
    public final XboxController hidDriver1, hidDriver2;
    
        private SendableChooser<Command> autoChooser;
    
        private RobotContainer() {
            pdp = new PowerDistribution();
    
            commandDriver1 = new CommandXboxController(0);
            hidDriver1 = commandDriver1.getHID();
            commandDriver2 = new CommandXboxController(1);
            hidDriver2 = commandDriver2.getHID();
    
            configureBindings();
            configureAutonomous();
            SmartDashboard.putData("Field", new Field2d());
        }
    
        private void configureBindings() {}
    
        public Command getAutonomousCommand() {
            return Commands.print("No autonomous command configured");
        }
        private void configureAutonomous() {
            AutoBuilder.configure(() -> new Pose2d(), pose -> {}, 
            () -> new ChassisSpeeds(),
            (ChassisSpeeds cs, DriveFeedforwards dff) -> {},
            new PathFollowingController() {
                @Override
                public boolean isHolonomic() {
                    // TODO Auto-generated method stub
                    return false;
                }
                @Override
                public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
                    // TODO Auto-generated method stub
                    
                }
                @Override
                public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose,
                        PathPlannerTrajectoryState targetState) {
                    // TODO Auto-generated method stub
                    return null;
                }
            },
      new RobotConfig(10, 10, new ModuleConfig(3, 3, 3, new DCMotor(0, 0, 0, 0, 0, 0), 3, 4), new Translation2d[] {new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()}     ),
      () -> false); // fix this with real swerve stuff

            
            autoChooser = AutoBuilder.buildAutoChooser();
            setupAutoDisplay();

            SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void setupAutoDisplay() {
        //update the displayed auto path in smartdashboard when ever the selection is changed
        //display is cleared in teleopInit
        if(autoChooser.getSelected() != null){}
            // LoggingManager.logValue("SelectedAuto", autoChooser.getSelected().getName());
        else{}
            // LoggingManager.logValue("SelectedAuto", "Null");
        
        autoChooser.onChange((selected) -> {
            if(DriverStation.isTeleopEnabled()) //don't display auton path in teleop
                return;

            displayAuto();

            if(autoChooser.getSelected() != null) {}
                // LoggingManager.logValue("SelectedAuto", autoChooser.getSelected().getName());
            else {}
                // LoggingManager.logValue("SelectedAuto", "Null");
        });

        /*
         * Robot.teleopInit clears the display
         * Robot.autonomousInit redraws the display
         */
    }
 public void displayAuto() {
        Command auto = autoChooser.getSelected();
        System.out.println(auto.getName());

        if(auto.getName().equals("InstantCommand")) {
            AutoDisplayHelper.clearAutoPath();
            return;
        }

        boolean isRed = false;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if(alliance.isPresent() && alliance.get() == Alliance.Red)
            isRed = true;

        AutoDisplayHelper.displayAutoPath(auto, isRed);
    }
}
