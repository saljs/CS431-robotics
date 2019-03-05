import lejos.robotics.RegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

/**
 * Trace a square 
 * @author Sal Skare
*/
public class SquareTracer
{
    public static void main(String[] args)
    {
        Wheel left = WheeledChassis.modelWheel(Motor.C, 49.6).offset(70);
        Wheel right = WheeledChassis.modelWheel(Motor.A, 49.6).offset(-70);
        Chassis chassis = new WheeledChassis(new Wheel[] { left, right }, WheeledChassis.TYPE_DIFFERENTIAL);
        MovePilot pilot = new MovePilot(chassis);
        LCD.drawString("Press ESC key", 0, 0);
        Button.LEDPattern(1);
        while(Button.ESCAPE.isUp()) {
            pilot.setLinearSpeed(200); // mm per second
            pilot.travel(620);         // mm
            pilot.rotate(-90);        // degree clockwise
        }

    }
}
