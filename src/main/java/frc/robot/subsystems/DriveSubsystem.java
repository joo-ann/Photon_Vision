package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MotorControllerConstants;

// Drive Subsystem stuff!
public class DriveSubsystem extends SubsystemBase {

  // All of the motor controllers
  WPI_TalonSRX m_rightDriveMotor1;
  WPI_TalonSRX m_rightDriveMotor2;
  WPI_TalonSRX m_leftDriveMotor1;
  WPI_TalonSRX m_leftDriveMotor2;

  // Differential Drive
  DifferentialDrive m_dT;

  // Constructor
  public DriveSubsystem() {

    // Pair controllers to CAN IDs
    m_rightDriveMotor1 = new WPI_TalonSRX(MotorControllerConstants.k_rightDriveMotor1ID);
    m_rightDriveMotor2 = new WPI_TalonSRX(MotorControllerConstants.k_rightDriveMotor2ID);
    m_leftDriveMotor1 = new WPI_TalonSRX(MotorControllerConstants.k_leftDriveMotor1ID);
    m_leftDriveMotor2 = new WPI_TalonSRX(MotorControllerConstants.k_leftDriveMotor2ID);

    // Set followers - Updated implementation for "MotorControllerGroup"
    m_rightDriveMotor2.follow(m_rightDriveMotor1);
    m_leftDriveMotor2.follow(m_leftDriveMotor1);
    
    // Pair Left and Right together to make a drivetrain!
    m_dT = new DifferentialDrive(m_leftDriveMotor1, m_rightDriveMotor1);
  }

  // Runs every scheduler run
  public void periodic() {
  }

  // Arcade Drive: Left Stick Drive, Right Stick Turn
  public void arcadeDrive(CommandXboxController driveController, double driveSpeed, double turnSpeed) {

    // Reversed! Controller, Turn, Drive. Not the other way around! Negative sign is to compensate for joystick inverts
    m_dT.arcadeDrive(
      driveController.getRawAxis(XboxController.Axis.kRightX.value) * turnSpeed, 
      driveController.getRawAxis(XboxController.Axis.kLeftY.value) * driveSpeed
    );
  }
}

