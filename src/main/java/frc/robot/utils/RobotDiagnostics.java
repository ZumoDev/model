package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

/**
 * Clase centralizada para los widgets CriticalLogs y VoltageHistory.
 * Gestiona errores de DS, monitoreo de batería, fallos de CAN y mensajes de sistema.
 */
public class RobotDiagnostics {
    private static RobotDiagnostics m_instance;
    private final List<String> m_logHistory = new ArrayList<>();
    private final int MAX_LOGS = 15;
    
    // Umbrales de batería
    private final double BATTERY_WARNING = 10.5; // Advertencia en log
    private final double BATTERY_CRITICAL = 9.0;  // Color crítico en widget de barras
    private boolean m_batteryWarningSent = false;

    public RobotDiagnostics() {
        // Mensaje de inicialización con timestamp del sistema
        addLog("SYSTEM_READY: Diagnostics Online", false);
    }

    /**
     * Singleton para acceder desde cualquier subsistema o manager sin pasar referencias.
     */
    public static RobotDiagnostics getInstance() {
        if (m_instance == null) {
            m_instance = new RobotDiagnostics();
        }
        return m_instance;
    }

    /**
     * Actualiza los widgets. Llamar en robotPeriodic().
     */
    public void update() {
        // 1. Obtener voltaje actual del controlador
        double voltage = RobotController.getBatteryVoltage();

        // 2. Publicar para el widget "VoltageHistory" (Gráfico de barras)
        // Llave: /SmartDashboard/BatteryVoltage
        SmartDashboard.putNumber("BatteryVoltage", voltage);

        // 3. Monitoreo lógico de batería para el Terminal de Logs
        checkBatteryLogic(voltage);

        // 4. Monitoreo de conexión con la Driver Station
        if (!DriverStation.isDSAttached()) {
            // Podrías añadir un log aquí, pero se llenaría el historial si el robot está apagado
        }

        // 5. Publicar al widget "CriticalLogs" (Terminal de texto)
        // Llave: /SmartDashboard/RobotErrors
        SmartDashboard.putStringArray("RobotErrors", m_logHistory.toArray(new String[0]));
    }

    /**
     * Añade un log personalizado.
     * @param msg Mensaje a mostrar.
     * @param isError Si es true, añade "ERR" para activar el color naranja en el widget de React.
     */
    public void addLog(String msg, boolean isError) {
        String timestamp = String.format("%.1f", Timer.getFPGATimestamp());
        String prefix = isError ? "ERR [" : "> [";
        String entry = prefix + timestamp + "] " + msg;

        // Insertar al inicio (index 0) para que lo más nuevo aparezca arriba
        m_logHistory.add(0, entry);

        // Limitar tamaño para no saturar NetworkTables ni la memoria
        if (m_logHistory.size() > MAX_LOGS) {
            m_logHistory.remove(m_logHistory.size() - 1);
        }
    }

    /**
     * Función específica para errores de CAN. Llamada desde CANManager.
     */
    public void reportCANFault(int id, String faultType) {
        addLog("CAN_FAULT ID:" + id + " [" + faultType + "]", true);
    }

    /**
     * Revisa el estado de la batería y genera alertas en el log si es necesario.
     */
    private void checkBatteryLogic(double voltage) {
        // Alerta de bajón crítico (Brownout potential)
        if (voltage < BATTERY_CRITICAL && !m_batteryWarningSent) {
            addLog("CRITICAL_VOLTAGE: " + String.format("%.1f", voltage) + "V - CHECK BATT!", true);
            m_batteryWarningSent = true;
        } 
        // Alerta de batería baja
        else if (voltage < BATTERY_WARNING && !m_batteryWarningSent) {
            addLog("LOW_BATTERY_WARNING: " + String.format("%.1f", voltage) + "V", true);
            m_batteryWarningSent = true; 
        } 
        // Resetear la bandera cuando el voltaje se estabilice (cambio de batería o fin de carga)
        else if (voltage > 12.0) {
            m_batteryWarningSent = false;
        }
    }
}