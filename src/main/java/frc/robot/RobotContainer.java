// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.PhotonVision;


// Lots of important stuff here! This is where the commands and subsystems come together
public class RobotContainer {
  double VISION_TURN_kP = 0.5;
  
  // New Command Xbox Controller (Command is for Command Based Stuff)
  CommandXboxController m_driverController = new CommandXboxController(DriverConstants.k_driverControllerPort);
  
  // Subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Vision
  PhotonVision m_vision = new PhotonVision(m_driverController, m_driveSubsystem);
  
  // Drive Commands
  private final ArcadeDriveCommand m_arcadeDefault = new ArcadeDriveCommand(m_driveSubsystem, 0.8, 0.6, m_driverController);

  // Constructor
  public RobotContainer() {

    // Sets the drive command as the default command
    m_driveSubsystem.setDefaultCommand(m_arcadeDefault);

    // Binds buttons to commands
    configureButtonBindings();
  }

  // Bind buttons and stuff here!
  private void configureButtonBindings() {
    new JoystickButton(m_driverController.getHID(), DriverConstants.k_A)
        .onTrue(new InstantCommand(() -> m_vision.TurnToTarget())
      );
  }

  // This is where you would put the PathPlanner stuff!
  public Command getAutonomousCommand() {
    return null;
  }
}