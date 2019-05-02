import lejos.hardware.sensor.HiTechnicCompass;
import lejos.utility.Delay;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.BaseRegulatedMotor;

class Callibrate {

    public static void main(String[] args) {
        HiTechnicCompass c = new HiTechnicCompass(SensorPort.S2);
        
        Motor.A.setSpeed(60);
        Motor.B.setSpeed(60);
        c.startCalibration();
        Motor.A.forward();
        Motor.B.backward();
        Delay.msDelay(40000);
        c.stopCalibration();
        Motor.A.stop();
        Motor.B.stop();
    }
}
