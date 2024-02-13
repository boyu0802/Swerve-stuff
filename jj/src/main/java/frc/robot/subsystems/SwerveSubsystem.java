// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import static frc.robot.Constants.ChassisConstants;



public class SwerveSubsystem extends SubsystemBase {

    public AHRS Navx;
    public SwerveDriveOdometry odometry;
    public SwerveDriveKinematics kinematics;
    public SwerveModule[] swerveModules;


    public SwerveSubsystem() {

        Navx = new AHRS(SerialPort.Port.kMXP);
        Navx.reset();


        kinematics = new SwerveDriveKinematics(
                new Translation2d(ChassisConstants.Chassis_Length / 2, ChassisConstants.Chassis_Width / 2),
                new Translation2d(-ChassisConstants.Chassis_Length / 2, ChassisConstants.Chassis_Width / 2),
                new Translation2d(-ChassisConstants.Chassis_Length / 2, -ChassisConstants.Chassis_Width / 2),
                new Translation2d(ChassisConstants.Chassis_Length / 2, -ChassisConstants.Chassis_Width / 2)
        );


        swerveModules = new SwerveModule[]{
                new SwerveModule(
                        RobotMap.Front_Left_Drive_Motor_ID,
                        RobotMap.Front_Left_Angle_Motor_ID,
                        RobotMap.Front_Left_Cancoder_ID,
                        RobotMap.Front_Left_Angle_Offset
                ),

                new SwerveModule(
                        RobotMap.Back_Left_Drive_Motor_ID,
                        RobotMap.Back_Left_Angle_Motor_ID,
                        RobotMap.Back_Left_Cancoder_ID,
                        RobotMap.Back_Left_Angle_Offset
                ),


                new SwerveModule(
                        RobotMap.Back_Right_Drive_Motor_ID,
                        RobotMap.Back_Right_Angle_Motor_ID,
                        RobotMap.Back_Right_Cancoder_ID,
                        RobotMap.Back_Right_Angle_Offset
                ),

                new SwerveModule(
                        RobotMap.Front_Right_Drive_Motor_ID,
                        RobotMap.Front_Right_Angle_Motor_ID,
                        RobotMap.Front_Right_Cancoder_ID,
                        RobotMap.Front_Right_Angle_Offset
                )
        };
        Timer.delay(1);

        resetModulesToAbsolute();
        odometry = new SwerveDriveOdometry(kinematics,getYaw(),getModulePosition());
    }

    public SwerveModulePosition[] getModulePosition(){
            return new SwerveModulePosition[]{
                    swerveModules[0].getPosition(),
                    swerveModules[1].getPosition(),
                    swerveModules[2].getPosition(),
                    swerveModules[3].getPosition()
            };
    }

//    public SwerveModuleState[] getModuleState(){
//        SwerveModuleState[] states = new SwerveModuleState[4];
//                for (SwerveModule module : swerveModules){
//                    module.getState();
//                }
//        return states;
//    }



    public void resetModulesToAbsolute(){
        for(var i = 0; i<4; i++) {
            swerveModules[i].resetToAbsolute();
//            SmartDashboard.putNumber("FLangleoffset",RobotMap.Front_Left_Angle_Offset.getDegrees());
//            SmartDashboard.putNumber("BLangleOffset", RobotMap.Back_Left_Angle_Offset.getDegrees());
//            SmartDashboard.putNumber("BRangleOffset", RobotMap.Back_Right_Angle_Offset.getDegrees());
//            SmartDashboard.putNumber("FRangleOffset",RobotMap.Front_Right_Angle_Offset.getDegrees());
        }
    }

//    public void resetOdometry(Pose2d pose){
//        odometry.resetPosition(getYaw(),getModulePosition(),pose);
//    }
//
//    public Pose2d getPose(){
//        return odometry.getPoseMeters();
//    }

    public void zeroGyro(){
        Navx.reset();
    }



    public Rotation2d getYaw(){
        return (Constants.NavxConstants.Navx_Inverted) ? Rotation2d.fromDegrees(360 - Navx.getYaw()): Rotation2d.fromDegrees(Navx.getYaw());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates( fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),-translation.getY(),rotation,getYaw()) : new ChassisSpeeds(translation.getX(),-translation.getY(),rotation));
            SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,ChassisConstants.Max_Speed);
            for(int i = 0; i< 4; i ++) {
                swerveModules[i].setState(moduleStates[i], isOpenLoop);
//                SmartDashboard.putNumber("set state" + i, moduleStates[i].angle.getDegrees());
            }
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePosition());
        for (var i = 0; i < 4; i++) {
             SmartDashboard.putNumber("CanCoder"+ i + "Degrees", swerveModules[i].getCancoder().getDegrees());
////            SmartDashboard.putNumber("CANCoder " + i + " Degrees", swerveModules[i].getCancoder().
            SmartDashboard.putNumber("AngleEncoder" + i + "Degrees", swerveModules[i].getAngle().getDegrees());
            SmartDashboard.putNumber("DriveEncoder" + i + "Meters", swerveModules[i].getState().speedMetersPerSecond);
        }
    }

}
