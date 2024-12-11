package frc.lib.tuning;

public interface TableSource extends ValueSource {
    int numRows();

    int numColumns();

    String getColumnName(int col);

    double getCellAsDouble(int row, int col);

    default int getCellAsInt(int row, int col) {
        return (int) getCellAsDouble(row, col);
    }

    void addListener(TableSourceListener listener);

    interface TableSourceListener {
        void onTableChange(int row, int col, double value);
    }
}
