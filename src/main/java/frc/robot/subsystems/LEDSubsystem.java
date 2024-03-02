package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.LedEffect;

public class LEDSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this LEDSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static LEDSubsystem INSTANCE = new LEDSubsystem();

    /**
     * Returns the Singleton instance of this LEDSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LEDSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LEDSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this LEDSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */

    private final int PWM_PORT = 0; // PWM port that the signal pin is connected to
    private final int STRIP_LENGTH = 38; // number of LEDs in each side strip
    private final AddressableLED led; // WIP's AddressableLED
    private final AddressableLEDBuffer ledBuffer; // WIP's AddressableLED
    private final int TPS = 50; // tick per second
    private int tick; // tick counter

    public LedEffect effect = () -> {};

    private static Color8Bit BLACK_COLOR = new Color8Bit(0,0,0);

    private LEDSubsystem() {
        this.led = new AddressableLED(PWM_PORT); // builds the LED strip
        this.ledBuffer = new AddressableLEDBuffer(STRIP_LENGTH); // builds the LED strip buffer
        this.led.setLength(this.ledBuffer.getLength()); //sets length

        this.tick = 0; // resets the tick counter

        effect = () ->
                blinkColor(1, new Color8Bit(0,0,255), new Color8Bit(255,0,0));
    }

    public void staticColor(Color8Bit color){
        for(int i = 0; i < ledBuffer.getLength(); i++){
            this.ledBuffer.setLED(i, color);
        }
    }


    // blinks all the LEDs between a color and black
    public void blinkColor(double frequency, Color8Bit color){
        blinkColor(frequency, color, BLACK_COLOR);
    }

    // blinks all the LEDs between two colors
    public void blinkColor(double frequency, Color8Bit color1, Color8Bit color2){
        int ticks_in_cycle = (int)(TPS/frequency);
        if (tick % ticks_in_cycle < ticks_in_cycle/2){
            staticColor(color1);
        }
        else {
            staticColor(color2);
        }
    }

    public void breathingColor(double frequency, Color8Bit color){
        double brightness = (1+Math.cos((tick*frequency/TPS)*2*Math.PI))/2;
        for(int i = 0; i < ledBuffer.getLength(); i++){
            this.ledBuffer.setRGB(i, (int)(color.red * brightness), (int)(color.green * brightness), (int)(color.blue * brightness));
        }
    }

    public void WaveColor(double frequency, boolean backward, Color8Bit color){
        if (tick%((int)(TPS/frequency)) != 0){ // skips runs
            return;
        }
        //for every led on the hopper
        for (int i = 0; i < ledBuffer.getLength(); i++){
            int red = (int)(ledBuffer.getLED(i).red * 255 * 0.75);
            int green = (int)(ledBuffer.getLED(i).green * 255 * 0.75);
            int blue = (int)(ledBuffer.getLED(i).blue * 255 * 0.75);

            ledBuffer.setRGB(i, red, green, blue); // dims every led to 75%
        }
        // int index = (tick/((int)(TPS/frequency)))%ledBuffer.getLength();
        // if (backward){
        //     index = ledBuffer.getLength() - 1 - index;
        // }
        // ledBuffer.setLED(index, color); // sets a new LED to full brightness
        int start = ((int)(tick/(TPS/frequency)))%5;
        if (backward) {
            for (int i = ledBuffer.getLength()-start-1; i < 0; i-=5) {
                ledBuffer.setLED(i, color);
            }
        }
        else{
            for (int i = start; i < ledBuffer.getLength(); i+=5) {
                ledBuffer.setLED(i, color);
            }
        }
        
    }

    private void errorEffect(){
        for(int i = 0; i < (ledBuffer.getLength()/2)-1; i++){
            this.ledBuffer.setLED(i, Color.kBlue);
        }
        for(int i = ledBuffer.getLength()/2; i < ledBuffer.getLength(); i++){
            this.ledBuffer.setLED(i, Color.kRed);
        }
    }

    @Override
    public void periodic() {
        try{
            effect.updateBuffer();
        }
        catch (Exception exception){
            errorEffect();
        }
        led.setData(ledBuffer);
        led.start();
        tick++;
    }
}

