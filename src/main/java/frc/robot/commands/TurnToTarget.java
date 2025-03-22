package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTarget extends Command{
    private DriveSubsystem m_driveSubsystem;
    private Camera m_camera;
    Timer timer = new Timer();

    public TurnToTarget(DriveSubsystem driveSubsystem, Camera camera, CommandXboxController xboxController) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
        m_camera = camera;
    }

    public void initialize() {};

    public void execute() {
        m_camera.TurnToTarget();
    }

    public boolean isFinished() {
        return timer.hasElapsed(5.0);
    }
}
