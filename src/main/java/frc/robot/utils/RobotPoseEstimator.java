package frc.robot.utils;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Importante: Asegúrate de tener el archivo LimelightHelpers.java en tu proyecto
import frc.robot.LimelightHelpers; 

public class RobotPoseEstimator {
    private final SwerveDrivePoseEstimator poseEstimator; //OBJETO QUE GUARDARA TODA LA POCISION DE LA SWERVE

    // Ahora pedimos la cinemática como parámetro para no tener que buscarla en constantes
    public RobotPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d robotAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, //Donde estan las llantas
            robotAngle, //Angulo inicial
            modulePositions, //Distancia que han recorrido las ruedas
            new Pose2d() //ASUME QUE ESTA EN LA POSE 0 CON ANGULO 0 
        );
    }

    public void update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(gyroAngle, modulePositions); //OBTIENE LA ODOMETRIA DE LOS ENCODERS Y PIGEON

        // Lógica de Limelight (Si tienen el archivo LimelightHelpers)
        // Usamos LimelightHelpers para verificar si hay un AprilTag válido en la mira
        if (LimelightHelpers.getTV("limelight")) { 
            // Obtenemos la pose del robot respecto al origen del campo (Azul)
            Pose2d visionPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            
            // Calculamos el timestamp exacto restando la latencia de captura y procesamiento
            // Esto evita que el robot "salte" si se está moviendo rápido
            double latency = (LimelightHelpers.getLatency_Pipeline("limelight") + LimelightHelpers.getLatency_Capture("limelight")) / 1000.0;
            double timestamp = Timer.getFPGATimestamp() - latency;

            // Inyectamos la medición de visión al estimador
            poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }

        Pose2d currentPose = poseEstimator.getEstimatedPosition(); //GUARDA LA VARIABLE
        double[] poseArray = {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()};//CREAMOS UN ARREGLO CON LOS VALORES NECESARIOS PARA EL FIELD
        SmartDashboard.putNumberArray("Field/Robot", poseArray); //Lo publica en la dashboard
    }

    /**
     * Resetea la posición del robot. Útil para el inicio del autónomo.
     */
    public void resetPose(Pose2d newPose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(gyroAngle, modulePositions, newPose);
    }
}
/*Implementacion en robot container
 * 
 * // 1. En la declaración de variables
private final DashboardHandler m_dashHandler = new DashboardHandler();
private final RobotPoseEstimator m_poseEstimator;

// 2. En el constructor del Drivetrain
m_poseEstimator = new RobotPoseEstimator(
    this.getKinematics(), // Phoenix 6 ya lo tiene
    this.getRotation2d(), 
    this.getModulePositions()
);

// 3. En el periodic() del Drivetrain
@Override
public void periodic() {
    // Actualiza la posición 50 veces por segundo
    m_poseEstimator.update(this.getRotation2d(), this.getModulePositions());
}

// 4. Cuando eligen un autónomo
public void displayAutoPath(Trajectory traj) {
    m_dashHandler.updateAutoPath(traj);
}
    // En getAutonomousCommand()
public Command getAutonomousCommand() {
    // 1. Leemos qué string mandó tu Dashboard (ej. "score_4")
    String selectedAuto = m_dashHandler.getAutoChooser().getSelected();

    // 2. Aquí es donde tu programador de auto mapea el nombre con el comando
    switch (selectedAuto) {
        case "taxi_left":
            return m_drive.getTaxiLeftCommand(); 
        case "score_4":
            return m_drive.getScore4Command();
        default:
            return new WaitCommand(1); // "Do Nothing"
    }
}


En el autonomo en el execute pon:
m_dashHandler.updateTargetPose(trajectory.sample(timer.get()).poseMeters);
 */