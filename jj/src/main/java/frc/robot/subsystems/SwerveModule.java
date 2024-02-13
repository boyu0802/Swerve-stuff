package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversion;
import frc.lib.util.ModuleStates;
import frc.lib.util.SwerveTypeConstants;
import frc.robot.Constants;
import frc.robot.Constants.AngleConstants;


public class SwerveModule {
     private final VelocityVoltage velocity = new VelocityVoltage(0);
     private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

     private final CANSparkMax angleMotor;
     private final TalonFX driveMotor;
     private final CANcoder cancoder;

     private final Rotation2d angleOffset;

     private Rotation2d lastAngle;
     public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
         Constants.DriveConstants.Drive_Ks , Constants.DriveConstants.Drive_Kv, Constants.DriveConstants.Drive_Ka
     );

     public SwerveModule(int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset){

          angleMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
          driveMotor = new TalonFX(angleMotorID);
          cancoder = new CANcoder(cancoderID);
          this.angleOffset = angleOffset;
          lastAngle = getState().angle;
          configAngleMotor();
          configDriveMotor();
          configCanCoder();
          resetToAbsolute();
          
     }

     public void setSpeed(SwerveModuleState state, boolean isOpenLoop){
          if (isOpenLoop){
               driveDutyCycle.Output =  state.speedMetersPerSecond/Constants.ChassisConstants.Max_Speed; //doesn't change the output based on sensors
               driveMotor.setControl(driveDutyCycle);
          }else {
               velocity.Velocity = Conversion.MPSToRPS(state.speedMetersPerSecond,SwerveTypeConstants.SDSMK4I_L1().wheelCircumference);
               velocity.FeedForward = feedforward.calculate(state.speedMetersPerSecond);// feedforward can be closed loop as long as it sends a signal back.
               driveMotor.setControl(velocity);
          }

     }
     public void setAngle(SwerveModuleState state){
          Rotation2d angle = (Math.abs(state.speedMetersPerSecond)<= (Constants.ChassisConstants.Max_Speed*0.01)) ? lastAngle : state.angle;

          angleMotor.getPIDController().setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
          lastAngle = angle;
     }

     public void setState(SwerveModuleState swerveModuleState, boolean isOpenLoop){
           SwerveModuleState state = ModuleStates.optimized(swerveModuleState, getState().angle);
           setSpeed(state, isOpenLoop);
           setAngle(state);

     }
     public Rotation2d getCancoder(){
          return Rotation2d.fromDegrees(Conversion.cancoderToDegrees(cancoder.getAbsolutePosition().getValue()));
     }





     public Rotation2d getAngle(){
          return Rotation2d.fromDegrees(angleMotor.getEncoder().getPosition());
     }





     public void resetToAbsolute(){
          angleMotor.getEncoder().setPosition(getCancoder().getDegrees() - angleOffset.getDegrees());
//          SmartDashboard.putNumber("anglemotor set position to:", getCancoder().getDegrees() - angleOffset.getDegrees());
//          SmartDashboard.putNumber("angle offset: ", angleOffset.getDegrees() );
//          SmartDashboard.putNumber("cancoder degrees", getCancoder().getDegrees());

     }





     public SwerveModulePosition getPosition() {
          return new SwerveModulePosition(
                  Conversion.falconToMeters(SwerveTypeConstants.SDSMK4I_L1().driveGearRatio,SwerveTypeConstants.SDSMK4I_L1().wheelCircumference, driveMotor.getPosition().getValue()) , getAngle()
          );
     }

     public SwerveModuleState getState(){
          return new SwerveModuleState(Conversion.falconToMPS(driveMotor.getPosition().getValue(), SwerveTypeConstants.SDSMK4I_L1().wheelCircumference),getAngle());
     }
     public void configAngleMotor(){
          angleMotor.restoreFactoryDefaults();

          angleMotor.getPIDController().setP(SwerveTypeConstants.SDSMK4I_L1().angleKp);
          angleMotor.getPIDController().setI(SwerveTypeConstants.SDSMK4I_L1().angleKi);
          angleMotor.getPIDController().setD(SwerveTypeConstants.SDSMK4I_L1().angleKd);
//          angleMotor.getPIDController().setSmartMotionAllowedClosedLoopError(AngleConstants.Angle_Allowed_Err, 0 );
          angleMotor.getPIDController().setSmartMotionMaxAccel(AngleConstants.Angle_Max_Accel,0);
          angleMotor.getPIDController().setSmartMotionMaxVelocity(AngleConstants.Angle_Max_Velo, 0);
          angleMotor.enableVoltageCompensation(AngleConstants.Angle_Voltage_Compensation);
          angleMotor.setSmartCurrentLimit(AngleConstants.Angle_Current_limit);
          angleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
          angleMotor.setInverted(SwerveTypeConstants.SDSMK4I_L1().angleMotorInvert);
          angleMotor.getEncoder().setPositionConversionFactor(360/SwerveTypeConstants.SDSMK4I_L1().angleGearRatio);
//          angleMotor.getEncoder().setPosition(0);
          angleMotor.burnFlash();
     }

     public void configDriveMotor(){
          driveMotor.getConfigurator().apply(new TalonFXConfiguration());

          TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
          driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          driveMotor.setNeutralMode(NeutralModeValue.Brake);
          driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveTypeConstants.SDSMK4I_L1().driveGearRatio;
          driveMotorConfig.Slot0.kP = Constants.DriveConstants.Drive_Kp;
          driveMotorConfig.Slot0.kI = Constants.DriveConstants.Drive_Ki;
          driveMotorConfig.Slot0.kD = Constants.DriveConstants.Drive_Kd;
          driveMotorConfig.Slot0.kV = Constants.DriveConstants.Drive_Kv;
          driveMotorConfig.Slot0.kA = Constants.DriveConstants.Drive_Ka;
          driveMotorConfig.Slot0.kS= Constants.DriveConstants.Drive_Ks;
          driveMotorConfig.CurrentLimits.SupplyTimeThreshold = Constants.DriveConstants.Drive_Supply_Time_Threshold;
          driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = Constants.DriveConstants.Drive_Supply_Current_Threshold;
          driveMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.DriveConstants.Drive_Suppy_Current_Limit;
          driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.DriveConstants.Drive_Current_Limit_Enable;
//          driveMotor.setPosition(0);
          driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.DriveConstants.Drive_Closed_Loop_Ramp;
          driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.DriveConstants.Drive_Closed_Loop_Ramp;
          driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.DriveConstants.Drive_Open_Loop_Ramp;
          driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.DriveConstants.Drive_Open_Loop_Ramp;
          driveMotor.getConfigurator().apply(driveMotorConfig);
     }

     public void configCanCoder(){
          CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
          cancoderConfig.MagnetSensor.SensorDirection = SwerveTypeConstants.SDSMK4I_L1().CANCODERInvert;
          cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
//          SmartDashboard.putNumber("angle motor suppose to be 0",angleMotor.getEncoder().getPosition());
//          cancoderConfig.MagnetSensor.MagnetOffset = (angleMotor.getEncoder().getPosition())/360;
//          cancoder.setPosition(0);


          cancoder.getConfigurator().apply(cancoderConfig);
     }





}


