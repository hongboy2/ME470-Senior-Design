import RPi.GPIO as GPIO
import time

# Configure GPIO settings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define the GPIO pin connected to the limit switch
LIMIT_SWITCH_PIN = 18

# Configure the GPIO pin as input with a pull-up resistor
GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def limit_switch_callback(channel):
    if GPIO.input(channel) == GPIO.LOW:
        print("Limit switch triggered")

try:
    # Add an event listener for the limit switch (both rising and falling edges)
    GPIO.add_event_detect(LIMIT_SWITCH_PIN, GPIO.BOTH, callback=limit_switch_callback, bouncetime=300)

    # Main loop
    while True:
        # Do other tasks here or simply sleep
        time.sleep(1)

except KeyboardInterrupt:
    print("Interrupted by user. Exiting...")

finally:
    # Cleanup GPIO settings before exiting
    GPIO.cleanup()
