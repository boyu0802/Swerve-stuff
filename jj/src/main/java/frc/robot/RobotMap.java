package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotMap {
    public static final int Front_Left_Drive_Motor_ID = 1;
    public static final int Front_Left_Angle_Motor_ID =1;
    public static final int Front_Left_Cancoder_ID = 1;
    public static final Rotation2d Front_Left_Angle_Offset = Rotation2d.fromDegrees(220.781250);

    public static final int Front_Right_Drive_Motor_ID = 4;
    public static final int Front_Right_Angle_Motor_ID =4;
    public static final int Front_Right_Cancoder_ID = 4;
    public static final Rotation2d Front_Right_Angle_Offset = Rotation2d.fromDegrees((69.697266));

    public static final int Back_Right_Drive_Motor_ID = 3;
    public static final int Back_Right_Angle_Motor_ID =3;
    public static final int Back_Right_Cancoder_ID = 3;
    public static final Rotation2d Back_Right_Angle_Offset = Rotation2d.fromDegrees(280.283203);


    public static final int Back_Left_Drive_Motor_ID = 2;
    public static final int Back_Left_Angle_Motor_ID =2;
    public static final int Back_Left_Cancoder_ID = 2;
    public static final Rotation2d Back_Left_Angle_Offset = Rotation2d.fromDegrees((266.835938));
}
