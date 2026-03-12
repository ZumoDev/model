package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerManager {
    // Definimos el PDH (Módulo 1 es el estándar para REV)
    private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
    private final double[] m_currents = new double[24];

    public void update() {
        // 1. Obtener voltajes y corriente total
        double voltage = m_pdh.getVoltage();
        double totalCurrent = m_pdh.getTotalCurrent();

        // 2. Llenar el arreglo de los 24 canales
        for (int i = 0; i < 24; i++) {
            m_currents[i] = m_pdh.getCurrent(i);
        }

        // 3. Publicar a NetworkTables según las llaves de tu widget
        // Usamos la ruta "PowerDistribution/..." para agrupar
        SmartDashboard.putNumberArray("PowerDistribution/Currents", m_currents);
        SmartDashboard.putNumber("PowerDistribution/TotalCurrent", totalCurrent);
        SmartDashboard.putNumber("PowerDistribution/Voltage", voltage);
    }
}