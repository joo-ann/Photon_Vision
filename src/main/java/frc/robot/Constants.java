package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {
  
  // Drive stuff
  public class DriverConstants {
    public static final int k_driverControllerPort = 0;
    public static final int k_A = XboxController.Button.kA.value;
    public static final int k_startButtonID = XboxController.Button.kStart.value;
  }  
  
  // CAN IDs
  public class MotorControllerConstants {
    public static final int k_leftDriveMotor1ID = 1;
    public static final int k_leftDriveMotor2ID = 2;
    public static final int k_rightDriveMotor1ID = 3;
    public static final int k_rightDriveMotor2ID = 4;
  }
}
