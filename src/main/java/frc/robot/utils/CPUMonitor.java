package frc.robot.utils;

import java.io.RandomAccessFile;
import java.io.IOException;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class CPUMonitor {
    private long prevIdle = 0;
    private long prevTotal = 0;
    private RandomAccessFile statFile;
    private double lastValue = 0;
    private double lastUpdateTime = 0;

    public CPUMonitor() {
        if (RobotBase.isReal()) {
            try {
                // Abrimos el archivo en modo solo lectura ("r")
                statFile = new RandomAccessFile("/proc/stat", "r");
            } catch (IOException e) {
                statFile = null;
            }
        }
    }

    /**
     * Calcula la carga de CPU comparando el tiempo activo vs el tiempo total.
     * Limitamos la lectura a máximo una vez cada 100ms para no saturar el bus de datos.
     */
    public double getCPULoad() {
        if (!RobotBase.isReal() || statFile == null) {
            return 10.0 + Math.random() * 5.0; // Valor simulado en PC
        }

        double currentTime = Timer.getFPGATimestamp();
        // Solo recalculamos si han pasado al menos 100ms
        if (currentTime - lastUpdateTime < 0.1) {
            return lastValue;
        }

        try {
            statFile.seek(0); // Volver al inicio del archivo sin cerrarlo
            String line = statFile.readLine();
            if (line == null) return lastValue;

            // El formato es: cpu  user nice system idle iowait irq softirq ...
            String[] parts = line.split("\\s+");

            long user = Long.parseLong(parts[1]);
            long nice = Long.parseLong(parts[2]);
            long system = Long.parseLong(parts[3]);
            long idle = Long.parseLong(parts[4]);
            long iowait = Long.parseLong(parts[5]);
            long irq = Long.parseLong(parts[6]);
            long softirq = Long.parseLong(parts[7]);

            long idleAll = idle + iowait;
            long total = user + nice + system + idleAll + irq + softirq;

            long totalDiff = total - prevTotal;
            long idleDiff = idleAll - prevIdle;

            prevTotal = total;
            prevIdle = idleAll;

            if (totalDiff > 0) {
                lastValue = 100.0 * (totalDiff - idleDiff) / totalDiff;
            }
            
            lastUpdateTime = currentTime;
            return lastValue;

        } catch (Exception e) {
            return -1;
        }
    }
}