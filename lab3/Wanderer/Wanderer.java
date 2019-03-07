import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import java.lang.Math;

public class Wanderer {

    //Ultrasonic Sensors
    private EV3UltrasonicSensor forward;
    private EV3UltrasonicSensor backLeft;
    private EV3UltrasonicSensor backRight;

    //Ultrasonic Sensor Sample Providers
    private SampleProvider distForward;
    private SampleProvider distBackR;
    private SampleProvider distBackL;
    
    //Motors
    private BaseRegulatedMotor left;
    private BaseRegulatedMotor right;

    //Wandering
    private Vector wanderDir;
    private long lastHeading;
    private float magDecay;
    private float dirDecay;
    private float decayRate;
    
    public static void main(String[] args) {
        Wanderer pilot = new Wanderer((BaseRegulatedMotor)Motor.C, 
                        (BaseRegulatedMotor)Motor.A,
                        SensorPort.S1, SensorPort.S2,
                        SensorPort.S3, (float)0.9);

        //Kf Kt maxDist time
        pilot.drive((float)200.0, (float)100.0, (float)0.2, 5000);
    }

    /*
     * Constructor Method
     */
    public Wanderer(BaseRegulatedMotor l,
                    BaseRegulatedMotor r,
                    Port f, Port bL,
                    Port bR, float decay){
        this.left = l;
        this.right = r;
        
        this.forward = new EV3UltrasonicSensor(f);
        this.backLeft = new EV3UltrasonicSensor(bL);
        this.backRight = new EV3UltrasonicSensor(bR);

        this.distForward = forward.getDistanceMode();
        this.distBackR = backRight.getDistanceMode();
        this.distBackL = backLeft.getDistanceMode();

        this.lastHeading = 0;
        this.magDecay = (float)1.0;
        this.dirDecay = (float)1.0;
        this.decayRate = decay;
    }

    public void drive(float Kf, float Kt, float distMax, long time) {
        while(Button.ESCAPE.isUp()){
            Vector heading = avoid();
            wander(distMax, time);
            heading.Add(this.wanderDir);
            //move the robot along this heading
            float speedL, speedR;
            speedL = Kf * heading.magnitude;
            speedR = Kf * heading.magnitude;
            speedL += heading.direction * Kt;
            speedR -= heading.direction * Kt;
            left.setSpeed(Math.abs(speedL));
            right.setSpeed(Math.abs(speedR));
            if(speedL > 0) {
                left.backward();
            }
            else {
                left.forward();
            }
            if(speedR > 0) {
                right.backward();
            }
            else {
                right.forward();
            }
        }
        left.stop();
        right.stop();
    }
    
    /*
     * generates random heading
     * @param distMax: constraint for heading magnitude
     * @param time: number of ms to keep each heading for
     */
    private void wander(float distMax, long time) {
        if(System.currentTimeMillis() - this.lastHeading >= time) {
            //generate random vector
            this.wanderDir = new Vector((float)(Math.random() * 2 * Math.PI * this.dirDecay),
                                        (float)(Math.random() * distMax * this.magDecay));
            this.dirDecay *= this.decayRate;
            this.magDecay *= this.decayRate;
        }
    }
    
    /*
     * Performs vector sum to generate heading based on
     *  ultrasonic sensor data
     */
    private Vector avoid() {
        float[] distanceSample = new float[this.distForward.sampleSize()];
        Vector heading = new Vector();
        
        //forward
        this.distForward.fetchSample(distanceSample,0);
        heading.Add(new Vector((float)0.0, averageDistance(distanceSample)));
    
        //back right (240 degrees)
        this.distBackR.fetchSample(distanceSample,0);
        heading.Add(new Vector((float)4.1888, averageDistance(distanceSample)));
        
        //back left (180 degress)
        this.distBackL.fetchSample(distanceSample,0);
        heading.Add(new Vector((float)2.0944, averageDistance(distanceSample)));
        
        return heading;
    }
    
    /*
     * calculates average distance
     * @param sample: sample of distance from sensor
     */
    private float averageDistance(float[] sample){
        float avg = 0;
        for (int ndx = 0; ndx < sample.length ; ndx++){
            avg += sample[ndx];
        }
        return avg/(sample.length);
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
        public Vector(float direction, float magnitide) {
            while(direction > Math.PI) {
                direction -= (float)(2 * Math.PI);
            }
            this.direction = direction;
            this.magnitude = magnitude;
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
            this.direction = (float)Math.atan2(y, x);
        }

    }
            
    
}  
