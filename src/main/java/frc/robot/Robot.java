// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.CPUMonitor;
import edu.wpi.first.wpilibj.RobotController;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final DashboardSubsystem dashboard = new DashboardSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private CommandSwerveDrivetrain drivetrain;
  private final CPUMonitor cpuMonitor=new CPUMonitor();
  private final RobotContainer m_robotContainer; 
  double timeLeft=1;

  //private final CPUMonitor m_CpuMonitor=new CPUMonitor();
  public Robot() {
    m_robotContainer = new RobotContainer();
    drivetrain = m_robotContainer.drivetrain;
  }

  @Override
  public void robotInit() {
    LimelightHelpers.setPipelineIndex(Constants.Limelights.shooterLimelightName, 1);

    //dashboard.sendPidData();

     timeLeft=Timer.getMatchTime();
  }

  @Override
  public void robotPeriodic() {
    //Temperaturas generales
    CommandScheduler.getInstance().run();
    double maxswerveTemp=drivetrain.getMaxSwerveTemp();
    double maxshooterTemp = shooterSubsystem.getShooterTemp();
    //double maxintakeTemp = m_robotContainer.m_intake.getIntakeTemp(); << FALTA POR DEFINIR
    //double[] temps = { maxSwerveTemp, maxShooterTemp, maxIntakeTemp };
    //SmartDashboard.putNumberArray("Temperatures", temps);
    //Tiempo de partido
    double timeLeft=Timer.getMatchTime();
    SmartDashboard.putNumber("TimeLeft", timeLeft);
    //HUB
    SmartDashboard.putBoolean("HubReady", m_robotContainer.isHubActive());
    //Estado de la Limelight:
    m_robotContainer.updateVisionDashboard();
    m_robotContainer.updateSystemHealth();
    double cpuLoad = cpuMonitor.getCPULoad();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    //Constants.Limelights.PID.Rotation.kP = dashboard.getPidData("swerve", "kP");
      
    //SmartDashboard.putBoolean("/SmartDashboard/HubReady", true);
  }
}
