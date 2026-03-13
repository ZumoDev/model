package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;

public class DashboardHandler {
    // 1. El selector de modos
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public DashboardHandler() {
        // Configuramos las opciones que aparecerán en tu select de React
        autoChooser.setDefaultOption("Do Nothing", "nothing");
        autoChooser.addOption("Taxi - Start Left", "taxi_left"); //en vez de taxi el nombre de del modo, me gusta ser un vago, un vago un vago jaja
        autoChooser.addOption("Score 4 Notes", "score_4"); //lo mismo aca que taxi
        
        // Esta línea publica 'options', 'selected' y 'active' automáticamente
        // bajo la ruta "/SmartDashboard/Auto Chooser/"
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Publica en donde debería de estar el robot
     */
    public void updateTargetPose(Pose2d targetPose){
        double[] poseArray = {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()};
        SmartDashboard.putNumberArray("Field/TargetPose", poseArray);
    }

    /**
     * Publica la trayectoria (AutoPath) para dibujar la línea morada
     */
    public void updateAutoPath(Trajectory trajectory) {
        // Extraemos los puntos de la trayectoria
        List<Trajectory.State> states = trajectory.getStates();
        // Creamos un arreglo: [x1, y1, rot1, x2, y2, rot2...]
        double[] pathData = new double[states.size() * 3];
        //Para que el path extraiga unicmanete x y y z (°)
        for (int i = 0; i < states.size(); i++) {
            pathData[i * 3] = states.get(i).poseMeters.getX();
            pathData[i * 3 + 1] = states.get(i).poseMeters.getY();
            pathData[i * 3 + 2] = states.get(i).poseMeters.getRotation().getDegrees();
        }

        // Se envía a la ruta que definiste en useEntry('/SmartDashboard/Field/AutoPath')
        SmartDashboard.putNumberArray("Field/AutoPath", pathData);
    }
}