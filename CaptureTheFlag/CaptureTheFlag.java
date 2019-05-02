import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;

import lejos.hardware.Sound;

import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

import java.lang.Math;
import java.lang.IllegalArgumentException;
import lejos.robotics.geometry.Rectangle2D;
import lejos.robotics.Color;

import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.hardware.sensor.HiTechnicCompass;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.device.NXTCam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Semaphore;

public class CaptureTheFlag {
    //Some constants
    private static final float CAMERA_WEIGHT    = (float)1.5;
    private static final float DISTANCE_WEIGHT  = (float)0.8;
    private static final float CLOSE_OBJ        = (float)0.2; //5cm
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

    //compass
    private SampleProvider compass;
    private int forwardDir;

    //Camera
    private int trackColor;
    private List<Rectangle2D> trackedObjects;
    private List<Integer> trackedColors;
    private Semaphore camLock;
    private Camera camThread;

    //Color sensor
    private SampleProvider color;

    public static void main(String[] args) {
        //setup
        CaptureTheFlag pilot = new CaptureTheFlag();
        Button.LEDPattern(RED);
        while(Button.ENTER.isUp());
        long startTime = System.currentTimeMillis();
        pilot.callibrate();
        Button.LEDPattern(AMBER);
        while(System.currentTimeMillis() - startTime < 5000);
        Button.LEDPattern(0);

        while(Button.ESCAPE.isUp() && !pilot.findBall());
        Button.LEDPattern(AMBER);
        pilot.grabBall();
        Button.LEDPattern(0);
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

        //set up color sensor
        EV3ColorSensor col = new EV3ColorSensor(SensorPort.S4);
        col.setFloodlight(Color.WHITE);
        this.color = col.getColorIDMode();

        //set up compass
        HiTechnicCompass com = new HiTechnicCompass(SensorPort.S2);
        this.compass = com.getCompassMode();
        
        //start camera thread
        this.camLock = new Semaphore(1);
        this.camThread = new Camera();
        this.camThread.start();
    }

    /*
     * Calibrates the robot sensors
     * @param red: Track red if true, blue if false
     */
    public void callibrate() {
        this.forwardDir = (int)readSensor(this.compass);
        this.arm.rotateTo(ARM_OPEN);
    }

    public boolean findBall() {
        Vector heading = new Vector((float)0.0, (float)1.0);
        
        //get ball heading
        int objects = 0;
        try {
            this.camLock.acquire();
            for(int i = 0; i < this.trackedObjects.size(); i++) {
                if(this.trackedColors.get(i) == RED_BALL) {
                    objects++;
                    Rectangle2D box = this.trackedObjects.get(i);
                    float cameraDir = (float)IMG_CENTER - (float)(box.getX() + ((box.getMaxX() - box.getX()) / 2.0));
                    heading.Add(new Vector((float)((cameraDir/IMG_CENTER)*Math.PI), CAMERA_WEIGHT));
                }
            }
            this.camLock.release();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(objects > 0)
            Button.LEDPattern(GREEN);
        else
            Button.LEDPattern(RED);
        
        //get ultrasonic heading
        if(readSensor(this.ultrasonic) < CLOSE_OBJ && objects == 0) {
            reverse(100);
            heading.Add(new Vector((float)-Math.PI, DISTANCE_WEIGHT));
        }
        
        //send commands to motors
        drive(heading);
        
        //return true if we managed to find the ball
        return ((int)readSensor(this.color) == Color.RED);
    }
    public void grabBall() {
        this.arm.rotateTo(ARM_CLOSED);
        reverse(2000);
        turnAround();
    }
    public boolean findGoal() {
        Vector heading = new Vector((float)0.0, (float)1.0);
        
        //get target heading
        int objects = 0;
        try {
            this.camLock.acquire();
            for(int i = 0; i < this.trackedObjects.size(); i++) {
                if(this.trackedColors.get(i) == BLUE_SCREEN) {
                    objects++;
                    Rectangle2D box = this.trackedObjects.get(i);
                    float cameraDir = (float)IMG_CENTER - (float)(box.getX() + ((box.getMaxX() - box.getX()) / 2.0));
                    heading.Add(new Vector((float)((cameraDir/IMG_CENTER)*Math.PI), CAMERA_WEIGHT));
                }
            }
            this.camLock.release();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(objects > 0)
            Button.LEDPattern(GREEN);
        else
            Button.LEDPattern(RED);
        
        //get ultrasonic heading
        if(readSensor(this.ultrasonic) < CLOSE_OBJ && objects == 0) {
            reverse(1500);
            heading.Add(new Vector((float)Math.PI, DISTANCE_WEIGHT));
        }

        //send commands to motors
        drive(heading);

        //return true if we reached the goal
        return (readSensor(this.ultrasonic) < CLOSE_OBJ && objects > 0);
    }
    public void stop() {
        left.stop();
        right.stop();
        this.camThread.finish();
    }
    public void reverse(long duration) {
        left.setSpeed(this.driveSpeed);
        right.setSpeed(this.driveSpeed);
        left.backward();
        right.backward();
        Delay.msDelay(duration);
        left.stop();
        right.stop();
    }
    public void turnAround() {
        int heading = 360 - this.forwardDir;
        left.setSpeed(this.driveSpeed);
        right.setSpeed(this.driveSpeed);
        left.forward();
        right.backward();
        while(Math.abs(readSensor(this.compass) - heading) > 5);
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

    class Camera extends Thread {
        private volatile boolean exit = false;
        public void run() {
            NXTCam cam = new NXTCam(SensorPort.S1);
            cam.setTrackingMode(NXTCam.OBJECT_TRACKING);
            cam.sortBy(NXTCam.SIZE);
            cam.enableTracking(true);
            trackedObjects = new ArrayList<>();
            trackedColors = new ArrayList<>();
            while(!exit) {
                try {
                    camLock.acquire();
                    trackedObjects = new ArrayList<>();
                    trackedColors = new ArrayList<>();
                    for(int i = 0; i < cam.getNumberOfObjects(); i++) {
                        trackedObjects.add(cam.getRectangle(i));
                        trackedColors.add(cam.getObjectColor(i));
                    }
                    camLock.release();
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        public void finish() {
            exit = true;
        }
    }
}
