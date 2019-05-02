import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;

import lejos.hardware.Sound;

import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

import java.lang.Math;
import java.lang.IllegalArgumentException;
import lejos.robotics.geometry.Rectangle2D;
import lejos.robotics.Color;

import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.device.NXTCam;

public class CaptureTheFlag {
    //Some constants
    private static final float COMPASS_WEIGHT   = (float)0.8;
    private static final float CAMERA_WEIGHT    = (float)1.5;
    private static final float DISTANCE_WEIGHT  = (float)3.0;
    private static final float CLOSE_OBJ        = (float)0.1; //5cm
    private static final long POLL_CAMERA       = (long)1000; //1sec
    private static final int IMG_CENTER = 80; //center of 160px image
    private static final int ARM_OPEN   = 70;
    private static final int ARM_CLOSED = -70;
    private static final int SPEED      = 250;

    //OBject colors
    private static final int RED_BALL       = 0;
    private static final int BLUE_BALL      = 1;
    private static final int RED_SCREEN     = 0;
    private static final int BLUE_SCREEN    = 2;
    
    //LED colors
    private static final int GREEN = 1;
    private static final int AMBER = 3;
    private static final int RED = 2;

    //Ultrasonic Sensor Sample Provider
    private SampleProvider ultrasonic;
    
    //Motors
    private BaseRegulatedMotor left;
    private BaseRegulatedMotor right;
    private BaseRegulatedMotor arm;

    //Speed
    private int driveSpeed;

    //Camera
    private NXTCam camera;
    private int trackColor;

    //Compass
    private SampleProvider compass;
    private int forwardDirection;

    //Color sensor
    private SampleProvider color;

    public static void main(String[] args) {
        //setup
        CaptureTheFlag pilot = new CaptureTheFlag();
        Button.LEDPattern(RED);
        //press Left for red and Right for blue
        while(Button.LEFT.isUp() && Button.RIGHT.isUp());
        long startTime = System.currentTimeMillis();
        pilot.callibrate(Button.LEFT.isDown());
        Button.LEDPattern(AMBER);
        while(System.currentTimeMillis() - startTime < 5000);
        Button.LEDPattern(0);

        while(Button.ESCAPE.isUp() && !pilot.findBall());
        pilot.grabBall();
        Button.LEDPattern(0);
        pilot.turnAround();
        while(Button.ESCAPE.isUp() && !pilot.findGoal());
        pilot.stop();
        Sound.beepSequenceUp();
    }

    /*
     * Constructor Method
     */
    public CaptureTheFlag(){
        //set up motors
        this.left = Motor.A;
        this.right = Motor.B;
        this.arm = Motor.C;
        this.driveSpeed = SPEED;

        //set up ultrasonic sensor
        NXTUltrasonicSensor usonic = new NXTUltrasonicSensor(SensorPort.S3);
        usonic.enable();
        this.ultrasonic = usonic.getDistanceMode();

        //set up compass sensor
        HiTechnicCompass comp = new HiTechnicCompass(SensorPort.S2);
        this.compass = comp.getAngleMode();

        //set up color sensor
        EV3ColorSensor col = new EV3ColorSensor(SensorPort.S4);
        col.setFloodlight(Color.WHITE);
        this.color = col.getColorIDMode();

        //set up camera
        this.camera = new NXTCam(SensorPort.S1);
        this.camera.setTrackingMode(NXTCam.OBJECT_TRACKING);
        //this.camera.sortBy(NXTCam.SIZE);
        this.camera.enableTracking(true);
    }

    /*
     * Calibrates the robot sensors
     * @param red: Track red if true, blue if false
     */
    public void callibrate(boolean red) {
        this.trackColor = red ? Color.RED : Color.BLUE;
        this.forwardDirection = (int)readSensor(this.compass);
        this.arm.rotateTo(ARM_OPEN);
        System.out.println(this.forwardDirection);
    }

    public boolean findBall() {
        Vector heading = new Vector();
        
        //get ball heading
        int objects = 0;
        int targetColor = (this.trackColor == Color.RED) ? RED_BALL : BLUE_BALL;
        for(int i = 0; i < camera.getNumberOfObjects(); i++) {
            if(this.camera.getObjectColor(i) == targetColor) {
                objects++;
                Rectangle2D box = this.camera.getRectangle(i);
                float cameraDir = (float)IMG_CENTER - (float)(box.getX() + ((box.getMaxX() - box.getX()) / 2.0));
                heading.Add(new Vector((float)((cameraDir/IMG_CENTER)*Math.PI), CAMERA_WEIGHT));
            }
        }
        if(objects > 0)
            Button.LEDPattern(GREEN);
        else
            Button.LEDPattern(RED);

        //get compass heading
        int compassDir = this.forwardDirection - (int)readSensor(this.compass);
        System.out.println(readSensor(this.compass));
        heading.Add(new Vector((float)((compassDir/180.0)*Math.PI), COMPASS_WEIGHT));

        /*/get ultrasonic heading
        if(readSensor(this.ultrasonic) < CLOSE_OBJ) {
            heading.Add(new Vector((float)Math.PI, DISTANCE_WEIGHT));
        }*/

        //send commands to motors
        drive(heading);
        
        //return true if we managed to find the ball
        return ((int)readSensor(this.color) == this.trackColor);
    }
    public void grabBall() {
        this.arm.rotateTo(ARM_CLOSED);
    }
    public void turnAround() {
        left.setSpeed(this.driveSpeed);
        right.setSpeed(this.driveSpeed);
        left.forward();
        right.backward();
        while(Math.abs(((this.forwardDirection+180) % 180) - (int)readSensor(this.compass)) > 5);
    }
    public boolean findGoal() {
        Vector heading = new Vector();
        
        //get target heading
        int objects = 0;
        int targetColor = (this.trackColor == Color.RED) ? RED_SCREEN : BLUE_SCREEN;
        for(int i = 0; i < camera.getNumberOfObjects(); i++) {
            if(this.camera.getObjectColor(i) == targetColor) {
                objects++;
                Rectangle2D box = this.camera.getRectangle(i);
                float cameraDir = (float)IMG_CENTER - (float)(box.getX() + ((box.getMaxX() - box.getX()) / 2.0));
                heading.Add(new Vector((float)((cameraDir/IMG_CENTER)*Math.PI), CAMERA_WEIGHT));
            }
        }
        if(objects > 0)
            Button.LEDPattern(GREEN);
        else
            Button.LEDPattern(RED);

        //get compass heading
        int compassDir = ((this.forwardDirection+180) % 180) 
                            - (int)readSensor(this.compass);
        heading.Add(new Vector((float)((compassDir/180.0)*Math.PI), COMPASS_WEIGHT));

        //send commands to motors
        drive(heading);

        //return true if we reached the goal
        return (readSensor(this.ultrasonic) < CLOSE_OBJ);
    }
    void stop() {
        left.stop();
        right.stop();
    }

    /*
     * Move the robot along this heading, relative to the robot
     * @param heading: The heading vector
     */
    private void drive(Vector heading) {
        float speedL = (float)(heading.getMagnitude() * this.driveSpeed)
            + (float)((heading.getDirection() / Math.PI) * this.driveSpeed);
        float speedR = (float)(heading.getMagnitude() * this.driveSpeed)
            - (float)((heading.getDirection() / Math.PI) * this.driveSpeed);
        left.setSpeed(Math.abs(speedL));
        right.setSpeed(Math.abs(speedR));
        if(speedL > 0) {
            left.forward();
        }
        else {
            left.backward();
        }
        if(speedR > 0) {
            right.forward();
        }
        else {
            right.backward();
        }
    }
    
    /*
     * Reads average value from a sensor
     * @param sensor: The sensor
     * @return The average sensor value
     */
    private float readSensor(SampleProvider sensor) {
        float[] sample = new float[sensor.sampleSize()];
        float avg = 0;
        sensor.fetchSample(sample, 0);
        for(int i = 0; i < sample.length; i++) {
            avg += sample[i];
        }
        return avg / sample.length;
    }  
    
    /*
     * Class for vectors
     */
    private class Vector {
        private float direction;
        private float magnitude;

        /*
         * Zero Constructor
         */
        public Vector() {
            this.direction = (float)0.0;
            this.magnitude = (float)0.0;
        }
        /*
         * Value Constructor
         */
        public Vector(float direction, float magnitude) {
            this.direction = (float)(direction % Math.PI);
            this.magnitude = magnitude;
        }

        /*
         * Copy constructor
         */
        public Vector(Vector v) {
            this.direction = v.getDirection();
            this.magnitude = v.getMagnitude();
        }

        /*
         * Add another vector to this vector
         * @param other: Vector to add
         */
        public void Add(Vector other) {
            //need to convert to rectangular coords.
            float x, y;
            x = (float)Math.cos(this.direction) * this.magnitude;
            y = (float)Math.sin(this.direction) * this.magnitude;
            x += (float)Math.cos(other.direction) * other.magnitude;
            y += (float)Math.sin(other.direction) * other.magnitude;
            this.magnitude = (float)Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            if(this.magnitude > 1.0)
                this.magnitude = (float)1.0;
            this.direction = (float)Math.atan2(y, x);
        }

        //Getters
        public float getMagnitude() {
            return this.magnitude;
        }
        public float getDirection() {
            return this.direction;
        }

        //Setters
        public void setMagnitude(float magnitude) {
            this.magnitude = magnitude;
        }
        public void setDirection(float direction) {
            this.direction = direction;
        }
    }
}
