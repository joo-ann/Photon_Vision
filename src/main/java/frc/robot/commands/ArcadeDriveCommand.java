package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

// Arcade Drive Command!
public class ArcadeDriveCommand extends Command {
  
  // Init stuff
  private DriveSubsystem m_driveSubsystem;
  private CommandXboxController m_driveController;
  private double m_driveSpeed;
  private double m_turnSpeed;

  // Constructor
  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, double driveSpeed, double turnSpeed, CommandXboxController driveController) {

    // Requirements equal to subsystem
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);

    // Speed controls
    m_driveSpeed = driveSpeed;
    m_turnSpeed = turnSpeed;
    m_driveController = driveController;
  }
  
  // Called when the command is initially scheduled.
  public void initialize() {
  }

  // Runs periodically
  public void execute() {
    m_driveSubsystem.arcadeDrive(m_driveController, m_turnSpeed, m_driveSpeed);
  }

  // We done yet?
  public boolean isFinished() {
    return false;
  }
}
