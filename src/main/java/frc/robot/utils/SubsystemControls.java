package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. This is made for the 2022 season
 */
public class SubsystemControls
{

    private final boolean limelight;
    private final boolean drive;
    
    /**
     * 
     * @param limelight
     *            indicates if the limelight subsystem is present and should be enabled
     * @param drive
     *            indicates if the drive subsystem is present and should be enabled
     */
    public SubsystemControls(
        @JsonProperty(required = true, value = "limelight")    boolean limelight,
        @JsonProperty(required = true, value = "drive")      boolean drive)
        
    {
        this.drive = drive;
        this.limelight = limelight;
    
    }

    public boolean isdrivePresent()
    {
        return drive;
    }
    public boolean islimelightPresent(){
        return limelight;
    }
    
    
}