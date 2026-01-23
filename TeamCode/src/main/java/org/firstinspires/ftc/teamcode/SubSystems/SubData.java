package org.firstinspires.ftc.teamcode.SubSystems;

public class SubData {

    public static boolean isRedTeam;
    public static double storedAngle = Math.PI;

    public enum OffsetIDs {
        GENERIC,
        BLUE_FAR,
        RED_FAR,
        BLUE_CLOSE,
        RED_CLOSE
    }

    // Offsets in radians
    static double GENERIC_STARTING_OFFSET    = Math.PI;
    static double BLUE_FAR_STARTING_OFFSET   = Math.PI * 1.50; // TODO: Test offsets
    static double RED_FAR_STARTING_OFFSET    = Math.PI * 0.50;
    static double BLUE_CLOSE_STARTING_OFFSET = Math.PI * 0.75;
    static double RED_CLOSE_STARTING_OFFSET  = Math.PI * 1.25;


    public void setTeam(boolean team) {
        isRedTeam = team;
    }

    public void toggleTeam() {
        isRedTeam = !isRedTeam;
    }

    public boolean isRedTeam() {
        return isRedTeam;
    }

    public static void setAngle(double angle) {
        storedAngle = angle;
    }

    public static double getAngle() {
        return storedAngle;
    }

    public static double getStartingAngleOffset(OffsetIDs startingPosition) {
        switch (startingPosition) {
            case BLUE_FAR:   return BLUE_FAR_STARTING_OFFSET;
            case RED_FAR:    return RED_FAR_STARTING_OFFSET;
            case BLUE_CLOSE: return BLUE_CLOSE_STARTING_OFFSET;
            case RED_CLOSE:  return RED_CLOSE_STARTING_OFFSET;
            case GENERIC:
            default:         return GENERIC_STARTING_OFFSET;
        }
    }



}
