package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotMap {
    public static final int Front_Left_Drive_Motor_ID = 1;
    public static final int Front_Left_Angle_Motor_ID =1;
    public static final int Front_Left_Cancoder_ID = 1;
    public static final Rotation2d Front_Left_Angle_Offset = Rotation2d.fromDegrees(49.130859);

    public static final int Front_Right_Drive_Motor_ID = 4;
    public static final int Front_Right_Angle_Motor_ID =4;
    public static final int Front_Right_Cancoder_ID = 4;
    public static final Rotation2d Front_Right_Angle_Offset = Rotation2d.fromDegrees(48.691406);

    public static final int Back_Right_Drive_Motor_ID = 3;
    public static final int Back_Right_Angle_Motor_ID =3;
    public static final int Back_Right_Cancoder_ID = 3;
    public static final Rotation2d Back_Right_Angle_Offset = Rotation2d.fromDegrees(120.058594 );


    public static final int Back_Left_Drive_Motor_ID = 2;
    public static final int Back_Left_Angle_Motor_ID =2;
    public static final int Back_Left_Cancoder_ID = 2;
    public static final Rotation2d Back_Left_Angle_Offset = Rotation2d.fromDegrees(87.451172);
}
