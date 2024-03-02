package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.LEDSubsystem;

public class LEDController {

    public static final Color8Bit BLUE_ALLIANCE_COLOR = new Color8Bit(0x0, 0x6d, 0xfe);
    public static final Color8Bit RED_ALLIANCE_COLOR = new Color8Bit(255, 0, 0);
    public static final Color8Bit WHITE = new Color8Bit(100, 100, 100);
    public static final Color8Bit ORANGE = new Color8Bit(255, 100, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
    private static Color8Bit allianceColor = WHITE;

    public enum StorageStates{
        EMPTY, HOLDING_PIECE;
    }

    public enum ActionStates{
        DEFAULT, FLOOR_COLLECTING, SOURCE_COLLECTING,
        AMP_AIMING, AMP_SHOOTING,
        SPEAKER_AIMING, SPEAKER_SHOOTING,
        OPENING_CLIMBER, CLOSING_CLIMBER;
    }

    private static StorageStates storageState = StorageStates.EMPTY;
    private static ActionStates actionState = ActionStates.DEFAULT;

    public static void setAllianceColor(Color8Bit color){
        LEDController.allianceColor = color;
    }

    public static void setStorageState(StorageStates state){
        LEDController.storageState = state;
    }

    public static void setActionState(ActionStates state){
        actionState = state;
    }

    public static void updateLEDEffect() {
        switch (actionState){
            case DEFAULT: {
                if (DriverStation.isDisabled()){
                    LEDSubsystem.getInstance().effect = () -> {};
                    LEDSubsystem.getInstance().staticColor(allianceColor);
                }
                else if (storageState == StorageStates.HOLDING_PIECE){
                    LEDSubsystem.getInstance().effect = () ->
                            LEDSubsystem.getInstance().blinkColor(2, ORANGE);
                }
                else if(DriverStation.isAutonomousEnabled()){
                    LEDSubsystem.getInstance().effect = () ->
                            LEDSubsystem.getInstance().breathingColor(1, allianceColor);
                }
                else if (DriverStation.isTeleopEnabled()){
                    LEDSubsystem.getInstance().effect = () ->
                            LEDSubsystem.getInstance().breathingColor(2, allianceColor);
                }
                break;
            }
            case FLOOR_COLLECTING:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().WaveColor(20, true, ORANGE);
                break;
            case SOURCE_COLLECTING:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().WaveColor(20, false, allianceColor);
                break;
            case AMP_AIMING:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().blinkColor(1, GREEN);
                break;
            case AMP_SHOOTING:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().WaveColor(20, false, GREEN);
                break;
            case SPEAKER_AIMING:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().blinkColor(2, GREEN);
                break;
            case SPEAKER_SHOOTING:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().WaveColor(40, true, GREEN);
                break;
            case OPENING_CLIMBER:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().WaveColor(40, false, WHITE);
                break;
            case CLOSING_CLIMBER:
                LEDSubsystem.getInstance().effect = () ->
                        LEDSubsystem.getInstance().WaveColor(40, true, WHITE);
                break;
        }
    }
}
