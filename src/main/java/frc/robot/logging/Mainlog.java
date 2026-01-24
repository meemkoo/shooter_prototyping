package frc.robot.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Mainlog {
    DataLog log = DataLogManager.getLog();

    public StringLogEntry legacyStateLog = new StringLogEntry(log, "/legacy/state");

    public Mainlog() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }
}
