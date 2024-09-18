from ev3dev2.led import Leds

from time import sleep


class Robo(MoveTank):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def cowardice:
        leds = Leds()
        leds.all_off() # Turn all LEDs off
        sleep(1)
        # Set both pairs of LEDs to amber
        leds.set_color('LEFT', 'AMBER')
        leds.set_color('RIGHT', 'AMBER')
        sleep(4)
        leds.set_color('LEFT', brightness_pct=0.5, trigger='timer')
        leds.set_color('RIGHT', brightness_pct=0.3, trigger='timer')
        leds.animate_flash('AMBER', sleeptime=0.25, duration=10)
        sleep(4)
        leds.all_off()
    
    def aggression:
        leds = Leds()
        leds.all_off() # Turn all LEDs off
        sleep(1)
        # Set both pairs of LEDs to amber
        leds.set_color('LEFT', 'AMBER')
        leds.set_color('RIGHT', 'AMBER')
        sleep(4)
        i = 0
        while i < 5 :
            leds.set_color('LEFT', brightness_pct= leds.max_brightness)
            leds.set_color('RIGHT', brightness_pct= leds.max_brightness)
            if i == 4:
                leds.animate_flash('AMBER', sleeptime=0.05, duration=3)



if __name__ == "__main__":
    robo = Robo(OUTPUT_B, OUTPUT_A)