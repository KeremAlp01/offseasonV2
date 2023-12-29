// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Trajectory {
    private DriveTrain drivetrain;
    private Trajectory[] trajectory = new Trajectory[2];

        public Trajectory(DriveTrain drivetrain){
            this.drivetrain = drivetrain;


            var DifferentialDriveVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(0, 0, 0,0,0));//ks,ka,,kv,kinematics,maxvoltage

            TrajectoryConfig configforward = new TrajectoryConfig(0, 0).setKinematics(drivetrain :: getKinematics).addConstaint(DifferentialDriveVoltageConstraint);

            TrajectoryConfig configbackward= new TrajectoryConfig(0, 0).setKinematics(drivetrain :: getKinematics).addConstaint(DifferentialDriveVoltageConstraint);

            trajectory[0] = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, new Rotation2d(0))),new Pose2d(7, 0, new Rotation2d(0)),configforward);
      



        }


        public RamseteCommand getRamsete(Trajectory trajectory){
            return new RamseteCommand(
                trajectory, drivetrain ::getPose2D,
                new RamseteController(0, 0),
                drivetrain :: getFeedForward,
                drivetrain :: getKinematics,
                drivetrain :: wheelSpeeds,
                new PIDController(0.85, 0, 0),
                new PIDController(0.85, 0, 0),
                null,
                 drivetrain);
            //trajectory,pose,controller,feedforward,kinematics,wheelspeeds,leftController(PID),rightController(PID),outputVolts,requirements
        }
}





