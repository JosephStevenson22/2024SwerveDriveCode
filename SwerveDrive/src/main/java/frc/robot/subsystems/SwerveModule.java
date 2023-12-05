package frc.robot.subsystems;

import javax.swing.text.Position;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    // Motor and encoder instances
    private final CANSparkMax turningMotor;
    private final TalonFX driveMotor;
    
    
    //Encoder Instand
    private final RelativeEncoder turnEncoder;


    
    // PID controller for turning
    private final PIDController turningPidController;

    //Encoder instance
    private final AnalogInput absoluteEncoder;
      
    // Configuration parameters
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
    {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushed);

        turnEncoder = turningMotor.getAlternateEncoder(498);
        
        
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        

        resetEncoders();
    
        
        
        

    }
    


    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }
    
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < .001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        SmartDashboard.putNumber("Encoder"+ absoluteEncoder.getChannel(), turnEncoder.getPosition());
        SmartDashboard.putNumber("EncoderVelocity"+ turningMotor.getDeviceId(), turnEncoder.getVelocity());
    }
        
    

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }
    

}