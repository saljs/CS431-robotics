import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.Button;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.device.NXTCam;
import lejos.robotics.geometry.Rectangle2D;
import lejos.utility.Delay;

/**
 * Image following robot driver
 * @author Sal Skare
*/
public class ImgFollower
{
    //Weighting variables
    private float Kp;
    private float Ki;
    private float Kd;
    private float damp;
    
    //Target center value
    private float target;

    //Motors
    private BaseRegulatedMotor left;
    private BaseRegulatedMotor right;
    
    //Camera
    private NXTCam camera;

    //LCD
    TextLCD screen;
    
    public static void main(String[] args)
    {
        ImgFollower pilot = new ImgFollower((BaseRegulatedMotor)Motor.C, 
                                            (BaseRegulatedMotor)Motor.A,
                                            SensorPort.S1);
        pilot.setWeights((float)0.5, (float)0.2, (float)0.1);
        pilot.setIntegralDampening((float)0.75);
        pilot.drive(180);
    }

    /*
     * Constructor method
     * @param left: Left motor
     * @param right: Right motor
     * @param sensor: Port that camera is connected to
     */
    public ImgFollower(BaseRegulatedMotor left, 
                       BaseRegulatedMotor right, 
                       Port sensor) {
        this.left = left;
        this.right = right;
        this.Kp = 1;
        this.Ki = 1;
        this.Kd = 1;
        this.damp = 1;
        this.camera = new NXTCam(sensor);
        this.camera.setTrackingMode('B');
        this.camera.sortBy('A');
        this.camera.enableTracking(true);
        this.target = (float)80.0; //Center of 160 pixel img
        final EV3 ev3 = (EV3) BrickFinder.getLocal();
        this.screen = ev3.getTextLCD();
    }

    /*
     * Set integral dampening
     * @param damp: Dampening constant
     */
    public void setIntegralDampening(float damp) {
        this.damp = damp;
    }

    /*
     * Reads the tracked object from camera
     * @return the pixel number of the box center [0, 88]
     */
    private float getBoxCenter() {
        float center = 0;
        int i;
        for(i = 0; i < camera.getNumberOfObjects(); i++) {
            Rectangle2D box = this.camera.getRectangle(i);
            center += box.getX() + ((box.getMaxX() - box.getX()) / 2.0);
        }
        center /= i;

        this.screen.clear();
        this.screen.drawString("center: " + center, 0, 1);
        return center;
    }
    /*
     * Reads the camera and gets box size
     * @return the size of the box in pixels
     */
    private float getBoxSize() {
        Rectangle2D box = this.camera.getRectangle(0);
        float size = (float)box.getHeight();
        this.screen.drawString("size: " + size, 0, 2);
        return size;
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
     * Starts the main drive loop
     * @param speed: Speed in degrees per second for the wheels to move at
     * @param finish: Width to stop at
     */
    public void drive(float speed) {
        float K, turn, error, lastErr = 0, deriv, integral = 0;
        int closeCount = 0;
        left.setSpeed(speed);
        right.setSpeed(speed);
        left.forward();
        right.forward();

        //main PID loop
        while(Button.ESCAPE.isUp()) {
            error = this.target - getBoxCenter();
            if(getBoxSize() > 140)
                closeCount++;
            if(closeCount > 3)
                break;
            if(Float.isNaN(error)) {
                error = 20;
            }
            integral = (this.damp * integral) + error;
            deriv = error - lastErr;
            turn = (this.Kp * error) + (this.Ki * integral) + (this.Kd * deriv);
            
            left.setSpeed(speed + turn);
            right.setSpeed(speed - turn);
            left.forward();
            right.forward();
            lastErr = error;

            this.screen.drawString("err: " + error, 0, 3);
            this.screen.drawString("turn: " + turn, 0, 4);
            Delay.msDelay(1);
        }

        //stop
        left.stop();
        right.stop();
    }
            
}                        

