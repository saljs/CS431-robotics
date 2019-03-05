import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.Button;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

/**
 * Line following robot driver
 * @author Sal Skare
*/
public class LineFollower
{
    //Weighting variables
    private float Kp;
    private float Ki;
    private float Kd;
    private float damp;
    
    //Target light value
    private float target;

    //Sensor read time difference
    private long lastSensorRead;
    private long dt;
    
    //Motors
    private BaseRegulatedMotor left;
    private BaseRegulatedMotor right;
    
    //Photosensor
    private EV3ColorSensor colorSensor;
    private SensorMode color;
    
    public static void main(String[] args)
    {
        LineFollower pilot = new LineFollower((BaseRegulatedMotor)Motor.C, 
                                              (BaseRegulatedMotor)Motor.A,
                                              SensorPort.S3);
        pilot.setWeights(500, 50, 10);
        pilot.setIntegralDampening((float)0.75);
        pilot.drive(300);
    }

    /*
     * Constructor method
     * @param left: Left motor
     * @param right: Right motor
     * @param sensor: Port that photosensor is connected to
     */
    public LineFollower(BaseRegulatedMotor left, 
                        BaseRegulatedMotor right, 
                        Port sensor) {
        this.left = left;
        this.right = right;
        this.Kp = 1;
        this.Ki = 1;
        this.Kd = 1;
        this.damp = 1;
        this.lastSensorRead = 0;
        calibrate(sensor);
    }

    /*
     * Calibrates the photosensor
     * @param conn: Port that photosensor is connected to
     */
    private void calibrate(Port conn) {
        final EV3 ev3 = (EV3) BrickFinder.getLocal();
        TextLCD lcd = ev3.getTextLCD();

        this.colorSensor = new EV3ColorSensor(conn);
        this.color = colorSensor.getRedMode();
        float light = 0;
        
        long startTime = System.currentTimeMillis();
        lcd.drawString("Set line:", 0, 1);
        while(Button.ENTER.isUp()) {
            if(System.currentTimeMillis() - startTime > 100) {
                light = getLight();
                lcd.drawString("" + light, 0, 3);
            }
        }
        this.target = light;
        Delay.msDelay(500);
        startTime = System.currentTimeMillis();
        lcd.drawString("Set bkgrnd:", 0, 1);
        while(Button.ENTER.isUp()) {
            if(System.currentTimeMillis() - startTime > 100) {
                light = getLight();
                lcd.drawString("" + light, 0, 3);
            }
        }
        this.target += light;
        this.target /= 2.0;
        lcd.drawString("Target: " + this.target, 0, 1);
    }
    
    /*
     * Reads the average light level
     * @return The light value normalized to range [0, 1]
     */
    private float getLight() {
        float[] colorSample = new float[ this.color.sampleSize() ];
        long sensorRead = System.currentTimeMillis();
        float light = 0;
        this.color.fetchSample(colorSample, 0);
        for(int i = 0; i < colorSample.length; i++) {
            light += colorSample[i];
        }
        this.dt = sensorRead - this.lastSensorRead;
        this.lastSensorRead = sensorRead;
        return light / colorSample.length;
    }

    /*
     * Sets the weights for the PID controller
     * @param Kp: Weight for error term
     * @param Ki: Weight for integral term
     * @param Kd: Weight for derivative term
     */
    public void setWeights(float Kp, float Ki, float Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /*
     * Set integral dampening
     * @param damp: Dampening constant
     */
    public void setIntegralDampening(float damp) {
        this.damp = damp;
    }

    /*
     * Starts the main drive loop
     * @param speed: Speed in degrees per second for the wheels to move at
     */
    public void drive(float speed) {
        final EV3 ev3 = (EV3) BrickFinder.getLocal();
        TextLCD lcd = ev3.getTextLCD();
        
        float K, turn, error, lastErr = 0, deriv, integral = 0;
        left.setSpeed(speed);
        right.setSpeed(speed);
        left.forward();
        right.forward();

        //main PID loop
        while(Button.ESCAPE.isUp()) {
            error = this.target - getLight();
            integral = (this.damp * integral) + (error * this.dt);
            deriv = (error - lastErr) / this.dt;
            turn = (this.Kp * error) + (this.Ki * integral) + (this.Kd * deriv);
            left.setSpeed(speed + turn);
            right.setSpeed(speed - turn);
            lastErr = error;
            lcd.drawString("" + error, 0, 3);
            Delay.msDelay(1);
        }

        //stop
        left.stop();
        right.stop();
    }
            
}                        

