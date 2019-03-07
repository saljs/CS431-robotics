import lejos.robotics.RegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.MovePilot;
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
    private MovePilot pilot;

    //Wandering
    private Vector wanderDir;
    private long lastHeading;
    private float magDecay;
    private float dirDecay;
    private float decayRate;
    
    public static void main(String[] args) {
        Wheel left = WheeledChassis.modelWheel(Motor.C, 49.6).offset(70);
        Wheel right = WheeledChassis.modelWheel(Motor.A, 49.6).offset(-70);
        Chassis chassis = new WheeledChassis(new Wheel[] { left, right }, WheeledChassis.TYPE_DIFFERENTIAL);
        MovePilot pilot = new MovePilot(chassis);
        Wanderer control = new Wanderer(pilot,
                        SensorPort.S1, SensorPort.S2,
                        SensorPort.S3, (float)0.9);

        control.drive(200,5000);
    }

    /*
     * Constructor Method
     */
    public Wanderer(MovePilot pilot,
                    Port f, Port bL,
                    Port bR, float decay){
        
        this.pilot = pilot;

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

    public void drive(float speed, long time) {
        pilot.forward();
        while(Button.ESCAPE.isUp()){
            Vector heading = avoid();
            wander(speed, time);
            heading.Add(this.wanderDir);
            //move the robot along this heading
            pilot.setLinearSpeed(speed * heading.magnitude);
            pilot.rotate((180/Math.PI)*heading.direction);
        }
        pilot.stop();
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
        
        //forward (pulled 180 degrees)
        this.distForward.fetchSample(distanceSample,0);
        heading.Add(new Vector((float)3.14159, averageDistance(distanceSample)));
    
        //back right (pulled 300 degrees)
        this.distBackR.fetchSample(distanceSample,0);
        heading.Add(new Vector((float)5.23599, averageDistance(distanceSample)));
        
        //back left (pulled 60 degress)
        this.distBackL.fetchSample(distanceSample,0);
        heading.Add(new Vector((float)1.04720, averageDistance(distanceSample)));
        
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
