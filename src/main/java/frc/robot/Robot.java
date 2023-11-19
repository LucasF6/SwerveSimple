// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      new Translation2d(1, -1), // FR
      new Translation2d(1, 1),  // FL
      new Translation2d(-1, -1),// BR
      new Translation2d(-1, 1)  // BL
  );
  PS4Controller m_controller = new PS4Controller(0);
  NetworkTable m_moduleData = NetworkTableInstance.getDefault().getTable("Modules");
  DoubleArrayTopic m_setpointsTopic = m_moduleData.getDoubleArrayTopic("setpoints");
  DoubleArrayPublisher m_setpointsPublisher = m_setpointsTopic.publish();
  double[] m_setpoints = new double[8];

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(
        m_controller.getLeftY(), m_controller.getLeftX(), m_controller.getRightX()));
    for (int i = 0; i < 4; i++) {
      m_setpoints[2 * i] = states[i].angle.getDegrees();
      m_setpoints[2 * i + 1] = states[i].speedMetersPerSecond;
    }
    m_setpointsPublisher.accept(m_setpoints);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
