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
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.utils.CPUMonitor;
import edu.wpi.first.wpilibj.RobotController;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final DashboardSubsystem dashboard = new DashboardSubsystem();

  private final RobotContainer m_robotContainer; 
  private final CPUMonitor m_CpuMonitor=new CPUMonitor();
  public Robot() {
    m_robotContainer = new RobotContainer();
  }
  //METODO PARA EL HUB
  //METODO DE HUB
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // Si no hay alianza (robot desconectado), el Hub está "bloqueado"
    if (alliance.isEmpty()) return false;

    // En Autónomo el Hub SIEMPRE está activo para ambos
    if (DriverStation.isAutonomousEnabled()) return true;

    // Si no estamos en Teleop ni Auto, está inactivo
    if (!DriverStation.isTeleopEnabled()) return false;

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    // Si no ha llegado el GameData (común al inicio), asumimos activo por seguridad
    if (gameData.isEmpty()) return true;

    // Determinamos quién ganó el bono de AUTO según el GameData
    // 'R' significa que Roja ganó Auto, 'B' que Azul ganó Auto
    boolean redWonAuto = (gameData.charAt(0) == 'R');

    // Lógica de "Shift 1 Active" según tu alianza y quién ganó el auto
    boolean shift1Active;
    if (alliance.get() == Alliance.Red) {
        // Si soy Rojo, el Shift 1 es activo si yo (Rojo) NO perdí el auto contra Azul
        // Según la tabla: Si Red gana Auto, Red está Inactivo en Shift 1.
        shift1Active = !redWonAuto; 
    } else {
        // Si soy Azul, el Shift 1 es activo si Rojo ganó el Auto
        shift1Active = redWonAuto;
    }

    // Tiempos basados en la tabla de la imagen y documentación
    if (matchTime > 130) return true;          // Transition Shift: Siempre activo
    if (matchTime > 105) return shift1Active;  // Shift 1 (2:10 - 1:45)
    if (matchTime > 80)  return !shift1Active; // Shift 2 (1:45 - 1:20)
    if (matchTime > 55)  return shift1Active;  // Shift 3 (1:20 - 0:55)
    if (matchTime > 30)  return !shift1Active; // Shift 4 (0:55 - 0:30)
    
    return true; // End Game (0:30 - 0:00): Siempre activo
}
public void updateVisionDashboard() {
    // 1. Obtener la pose del objetivo relativa a la cámara (Camera Space)
    // El arreglo contiene: [x, y, z, roll, pitch, yaw]
    double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace("limelight");

    // 2. Validación de seguridad (si no hay target, el arreglo suele estar vacío o en 0)
    // tv es 1 si la Limelight ve un objetivo
    boolean hasTarget = LimelightHelpers.getTV("limelight");

    if (hasTarget && targetPose.length >= 3) {
        // Extraemos las coordenadas 3D (en metros)
        double x = targetPose[0]; // Desplazamiento lateral (izquierda/derecha)
        double y = targetPose[1]; // Desplazamiento vertical (arriba/abajo)
        double z = targetPose[2]; // Profundidad (hacia adelante)

        // 3. LA FÓRMULA MAESTRA: Distancia Euclidiana 3D
        // Calcula la línea recta real desde el lente hasta el AprilTag
        double distance = Math.sqrt(
            Math.pow(x, 2) + 
            Math.pow(y, 2) + 
            Math.pow(z, 2)
        );

        // 4. Publicar al Widget (Llave: "TargetDistance")
        SmartDashboard.putNumber("TargetDistance", distance);
        
        // Opcional: Publicar TX para que el radar del widget se mueva
        SmartDashboard.putNumber("TargetTX", LimelightHelpers.getTX("limelight"));
        
    } else {
        // Si no hay target, mandamos 0 para que el widget muestre "---"
        SmartDashboard.putNumber("TargetDistance", 0.0);
    }
  }
    //METODO PARA EL CAN ANCHO DE BANDA, RIO TEMP
    public void updateSystemHealth() {
    // 1. CAN Bus Utilization
    var canStatus = RobotController.getCANStatus();
    // Enviamos decimal (0.0 a 1.0) porque tu widget hace (canUsage * 100)
    double canUsage = canStatus.percentBusUtilization / 100.0; 
    // En 2026 intentaron separar la carga del sistema de la carga del proceso
    double rioTemp = RobotController.getCPUTemp();

    // PUBLICACIÓN
    SmartDashboard.putNumber("CANUsage", canUsage);
    SmartDashboard.putNumber("RoboRIOTemp", rioTemp);
    }


  @Override
  public void robotInit() {
    LimelightHelpers.setPipelineIndex(Constants.Limelights.shooterLimelightName, 1);

    m_robotContainer.drivetrain.resetPose(
      new Pose2d(
        4, 3, new Rotation2d(0)
      )
    );

    dashboard.sendPidData();
  }

  @Override
  public void robotPeriodic() {
      //Temperaturas generales
    CommandScheduler.getInstance().run();
    //double maxswerveTemp=m_robotContainer.getMaxSwerveTemp();
    //double maxshooterTemp = m_robotContainer.m_shooter.getShootertemp(); 
    //double maxintakeTemp = m_robotContainer.m_intake.getIntakeTemp();
    //double[] temps = { maxSwerveTemp, maxShooterTemp, maxIntakeTemp };
    //SmartDashboard.putNumberArray("Temperatures", temps);
    //Match time
    double timeLeft=Timer.getMatchTime();
    SmartDashboard.putNumber("TimeLeft", timeLeft);
    //HUB
    SmartDashboard.putBoolean("HubReady", isHubActive());
    //Limelight status:
    updateVisionDashboard();
    updateSystemHealth();
    //double cpuLoad = m_cpuMonitor.getCPULoad();
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
    Constants.Limelights.PID.Rotation.kP = dashboard.getPidData("swerve", "kP");
        double fakeMatchtime= 10;
        SmartDashboard.putNumber("TimeLeft", fakeMatchtime);
        System.out.println("PRUEBA");
  }
}
