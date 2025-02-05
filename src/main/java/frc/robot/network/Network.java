package frc.robot.network;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;

public class Network {
    // Implement the singleton pattern
    private static Network instance = null;
    // The main object we use to interact with the NetworkTables
    private final NetworkTableInstance tableInstance;
    private final Map<String, Table> tableCache = new HashMap<>();

    private Network() {
        tableInstance = NetworkTableInstance.getDefault();
    }

    private static Network getInstance() {
        if (instance == null)
            instance = new Network();
        return instance;
    }

    public static Table getTable(String tableName) {
        return getInstance()._getTable(tableName);
    }

    public static void printAllEntriesFromTable(String tableName) {
        var table = getTable(tableName);
        var entries = table.getAllEntries();
        for (var entry : entries.entrySet()) {
            String entryKey = entry.getKey();
            String entryValue = entry.getValue();
            SmartDashboard.putString("table/" + tableName + ":" + entryKey, entryValue);
        }
    }

    private Table _getTable(String tableName) {
        if (tableCache.containsKey(tableName)) {
            return tableCache.get(tableName);
        }
        var res = new Table(tableName, tableInstance.getTable(tableName));
        tableCache.put(tableName, res);
        return res;
    }
}