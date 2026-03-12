package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

/**
 * Gestor central del Bus CAN.
 * Alimenta: CANBusMonitor, DetailedMotorCard (16 motores) y CriticalLogs.
 */
public class CANManager {
    private final Map<TalonFX, String> talons = new HashMap<>();
    private final Map<SparkMax, String> sparks = new HashMap<>();
    
    private String lastStickyFault = "NONE";
    private boolean hasSticky = false;

    /**
     * Registra un motor TalonFX con su ruta de telemetría.
     * @param motor Instancia del motor.
     * @param ntRoot Ruta en SmartDashboard (ej: "Drive/FL").
     */
    public void addMotor(TalonFX motor, String ntRoot) { talons.put(motor, ntRoot); }

    /**
     * Registra un motor SparkMax con su ruta de telemetría.
     * @param motor Instancia del motor.
     * @param ntRoot Ruta en SmartDashboard (ej: "Shooter/L").
     */
    public void addMotor(SparkMax motor, String ntRoot) { sparks.put(motor, ntRoot); }

    public void update() {
        var status = RobotController.getCANStatus();
        StringBuilder logBuilder = new StringBuilder();

        // 1. DATOS GLOBALES (Para SystemHealth y CANBusMonitor)
        SmartDashboard.putNumber("CANBus/Utilization", status.percentBusUtilization * 100);
        SmartDashboard.putNumber("CANBus/BusOffCount", status.busOffCount);
        SmartDashboard.putNumber("CANBus/TxErrors", status.txFullCount);
        SmartDashboard.putNumber("CANBus/RxErrors", status.receiveErrorCount);

        // 2. PROCESAR TALONFX (Telemetría + Errores)
        for (var entry : talons.entrySet()) {
            TalonFX m = entry.getKey();
            String path = "Drive/".contains(entry.getValue()) || "Steer/".contains(entry.getValue()) 
                          ? entry.getValue() : entry.getValue(); // Simplificado para usar el root directo
            
            String fullPath = "SmartDashboard/" + entry.getValue();

            // Telemetría para DetailedMotorCard
            SmartDashboard.putNumber(fullPath + "/Temp", m.getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber(fullPath + "/Current", m.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber(fullPath + "/Voltage", m.getMotorVoltage().getValueAsDouble());

            // Sticky Faults
            checkTalonFaults(m, logBuilder);
        }

        // 3. PROCESAR SPARKMAX (Telemetría + Errores)
        for (var entry : sparks.entrySet()) {
            SparkMax m = entry.getKey();
            String fullPath = "SmartDashboard/" + entry.getValue();

            // Telemetría para DetailedMotorCard
            SmartDashboard.putNumber(fullPath + "/Temp", m.getMotorTemperature());
            SmartDashboard.putNumber(fullPath + "/Current", m.getOutputCurrent());
            SmartDashboard.putNumber(fullPath + "/Voltage", m.getBusVoltage());

            // Sticky Faults
            checkSparkFaults(m, logBuilder);
        }

        // 4. LÓGICA DE RESET (Desde el botón de la Dashboard)
        if (SmartDashboard.getBoolean("CANBus/ClearStickyFaults", false)) {
            clearAllStickyFaults();
            SmartDashboard.putBoolean("CANBus/ClearStickyFaults", false);
            hasSticky = false;
            lastStickyFault = "NONE";
            logBuilder.setLength(0);
            RobotDiagnostics.getInstance().addLog("CAN_ERRORS_CLEARED", false);
        }

        // 5. ACTUALIZAR ESTADO DE ERRORES
        if (logBuilder.length() > 0) {
            hasSticky = true;
            lastStickyFault = logBuilder.toString();
        }

        SmartDashboard.putBoolean("CANBus/StickyFaults", hasSticky); // Para NotificationSystem
        SmartDashboard.putString("CANBus/Status", hasSticky ? lastStickyFault : "OK");
    }

    private void checkTalonFaults(TalonFX m, StringBuilder log) {
        boolean fault = false;
        if (m.getStickyFault_Undervoltage().getValue()) { log.append("T").append(m.getDeviceID()).append(":V; "); fault = true; }
        if (m.getStickyFault_Hardware().getValue()) { log.append("T").append(m.getDeviceID()).append(":HW; "); fault = true; }
        
        if (fault) RobotDiagnostics.getInstance().reportCANFault(m.getDeviceID(), "TALON_STUCK");
    }

    private void checkSparkFaults(SparkMax m, StringBuilder log) {
        var s = m.getStickyFaults();
        if (s.rawBits != 0) {
            log.append("S").append(m.getDeviceId()).append(":").append(s.rawBits).append("; ");
            RobotDiagnostics.getInstance().reportCANFault(m.getDeviceId(), "SPARK_STUCK_BITS_" + s.rawBits);
        }
    }

    private void clearAllStickyFaults() {
        talons.keySet().forEach(TalonFX::clearStickyFaults);
        sparks.keySet().forEach(SparkMax::clearFaults);
    }
}