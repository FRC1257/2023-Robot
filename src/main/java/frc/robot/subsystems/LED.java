package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import static frc.robot.Constants.LED.*;

public class LED extends SnailSubsystem {
	private AddressableLED led;
	private AddressableLEDBuffer buffer;

	public enum LEDState {
		YELLOW,
		PURPLE,
		NEUTRAL
	}

	private LEDState ledState;

	public LED() {
		led = new AddressableLED(LED_PORT_NUMBER);
		buffer = new AddressableLEDBuffer(LED_COUNT);
		led.setLength(LED_COUNT);
		led.setData(buffer);
		led.start();
		ledState = LEDState.NEUTRAL;
	}
	
	private void setAllRGB(int r, int g, int b) {
		for (int i = 0; i < LED_COUNT; i++) {
			buffer.setRGB(i, r, g, b);
		}
	}
	@Override
	public void update() {
		switch (ledState) {
			case YELLOW:
				setAllRGB(255, 255, 0);
				led.setData(buffer);
				break;
			case PURPLE:
				setAllRGB(255, 0, 255);
				led.setData(buffer);
				break;
			case NEUTRAL:
				setAllRGB(0, 0, 0);
				led.setData(buffer);
				break;
		}
	}

	public LEDState getState() {
		return ledState;
	}

	@Override
	public void displayShuffleboard() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void tuningInit() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void tuningPeriodic() {
		// TODO Auto-generated method stub
		
	}
	
	public void yellow() {
		ledState = LEDState.YELLOW;
	}

	public void purple() {
		ledState = LEDState.PURPLE;
	}
	
	public void neutral() {
		ledState = LEDState.NEUTRAL;
	}
}