package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class RobotMap {

    public static final int FRONT_LEFT_DRIVE_PORT = 1;
    public static final int BACK_LEFT_DRIVE_PORT = 3;
    public static final int FRONT_RIGHT_DRIVE_PORT = 2;
    public static final int BACK_RIGHT_DRIVE_PORT = 4;

    public static WPI_TalonSRX frontLeftDriveMotor = new WPI_TalonSRX(FRONT_LEFT_DRIVE_PORT);
    public static WPI_TalonSRX frontRightDriveMotor = new WPI_TalonSRX(FRONT_RIGHT_DRIVE_PORT);
    public static WPI_TalonSRX backLeftDriveMotor = new WPI_TalonSRX(BACK_LEFT_DRIVE_PORT);
    public static WPI_TalonSRX backRightDriveMotor = new WPI_TalonSRX(BACK_RIGHT_DRIVE_PORT);
}
