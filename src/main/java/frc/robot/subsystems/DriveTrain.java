// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax rightmotor =new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftmotor =new CANSparkMax(2, MotorType.kBrushless);
 

  private DifferentialDrive chassis = new DifferentialDrive(leftmotor, rightmotor);

  DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(0);

  private Field2d field = new Field2d();

  DifferentialDriveWheelSpeeds wheelSpeeds;
  ChassisSpeeds chassisSpeeds;
  double angularSpeed;
  double linearSpeed;

  private AHRS gyro = new AHRS(Port.kMXP);
  

  DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim( DCMotor.getNEO(1),
    7.29, //TODO change
    7.5,  //TODO change
    20,  //TODO change
    Units.inchesToMeters(6.0),  //TODO change
    Units.inchesToMeters(27.0),  //TODO change
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));  //TODO change

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), leftmotor.getEncoder().getPosition(), rightmotor.getEncoder().getPosition());
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243);



    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public DriveTrain() {
    leftmotor.getEncoder().setPositionConversionFactor(2 * Math.PI * 6 / 2048);
    rightmotor.getEncoder().setPositionConversionFactor(2 * Math.PI * 6 / 2048);

    REVPhysicsSim.getInstance().addSparkMax(leftmotor, DCMotor.getNEO(1));

    SmartDashboard.putData("Field", field);
    
  }


  public void drive(DoubleSupplier speed,DoubleSupplier rotation,boolean allowTurningPlace){
    chassis.curvatureDrive(speed.getAsDouble(), rotation.getAsDouble(), allowTurningPlace);
  }


  public SimpleMotorFeedforward getFeedForward(){
    return feedforward;
  }

  public Pose2d getPose2D(){
    return odometry.getPoseMeters();
  }
  
  public DifferentialDriveKinematics getKinematics(){
    return driveKinematics;
  }


  public DifferentialDriveWheelSpeeds wheelSpeeds(){
    return wheelSpeeds;
  }

  public double LeftWheelSpeed(){
    return wheelSpeeds.leftMetersPerSecond;
  }

  public double RightWheelSpeeds(){
    return wheelSpeeds.rightMetersPerSecond;
  }
  
  

  @Override
  public void periodic() {
    wheelSpeeds = new DifferentialDriveWheelSpeeds(leftmotor.getEncoder().getPosition(), rightmotor.getEncoder().getPosition());
    chassisSpeeds = driveKinematics.toChassisSpeeds(wheelSpeeds);


    linearSpeed = chassisSpeeds.vxMetersPerSecond;
    angularSpeed = chassisSpeeds.omegaRadiansPerSecond;
    

    
    odometry.update(getHeading(), leftmotor.getEncoder().getPosition(), rightmotor.getEncoder().getPosition());

   
    
    field.setRobotPose(odometry.getPoseMeters());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic(){
    REVPhysicsSim.getInstance().run();
    drivetrainSim.setInputs(leftmotor.get() * RobotController.getInputVoltage(),  rightmotor.get() * RobotController.getInputVoltage());

  
    drivetrainSim.update(0.02);

    leftmotor.getEncoder().setPosition(drivetrainSim.getLeftPositionMeters());
    leftmotor.getEncoder().setVelocityConversionFactor(drivetrainSim.getLeftVelocityMetersPerSecond());

    rightmotor.getEncoder().setPosition(drivetrainSim.getLeftPositionMeters());
    rightmotor.getEncoder().setVelocityConversionFactor(drivetrainSim.getRightVelocityMetersPerSecond());


  }
}
