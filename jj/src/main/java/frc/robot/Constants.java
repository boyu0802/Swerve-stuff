// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    public static class NavxConstants{
        public static boolean Navx_Inverted = true;
    }


    public static class ChassisConstants{
        public static final double Chassis_Length = 0.62865;
        public static final double Chassis_Width = 0.62865;

        public static final double Chassis_Wheel_Diameter_In_Meters = Units.inchesToMeters(4);
        public static final double Max_Speed = 4.5;
        public static final double Max_Voltage = 12.0;

        public static final boolean is_Open_Loop = true;


    }

    public static class DriveConstants{
        public static final double Drive_Ks = 0.32/ChassisConstants.Max_Voltage;
        public static final double Drive_Ka = 0.27/ChassisConstants.Max_Voltage;
        public static final double Drive_Kv = 1.51/ ChassisConstants.Max_Voltage;

        public static final double Drive_Kp =0.1;
        public static final double Drive_Ki = 0.0;
        public static final double Drive_Kd =0.0;

        public static final double Drive_Supply_Time_Threshold = 0.1;
        public static final double Drive_Supply_Current_Threshold = 60;
        public static final double Drive_Suppy_Current_Limit = 35;

        public static final boolean Drive_Current_Limit_Enable = true;
        public static final double Drive_Closed_Loop_Ramp = 0.0;
        public static final double Drive_Open_Loop_Ramp = 0.5;

    }

    public static class AngleConstants{

        public static final double Angle_Max_Accel =4.5 ;
        public static final double Angle_Max_Velo = 4.5;
//        public static final double Angle_Allowed_Err = .0;
        public static final double Angle_Voltage_Compensation = 12.0;
        public static final int Angle_Current_limit = 35;
    }
}
